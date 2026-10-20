/**
 * edited : 2026. 10. 07.
 * ----------------------------------------------------------------------------
 * License: Apache License 2.0
 * * Copyright [2025,2026] [Jehyeon Park, Suyung Park]
 * * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * * http://www.apache.org/licenses/LICENSE-2.0
 * * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ----------------------------------------------------------------------------
 * M7.ino - Portenta H7 M7 Core
 * ============================================================================
 * 재활 로봇 모터 제어 - 다중 모터 UDP 동시 제어 버전
 * 
 * - Jetson To M7 UDP 통신 도입 (Port 9999 수신)
 * - 3개 연결 모터(ID 110/130/120) 순차 제어 루프 (500Hz)
 *   [25차] 2026-08 리그 리디자인으로 CAN ID 전면 변경 (구 100/20/10 — can_id_scanner 실측)
 * - 손목 관절(j3): 2026-09-22 부터 AK60-6 direct-drive(ID 20) 로 활성 — MD80 임피던스 모드 (35차)
 * - 기존 모터 프로필(motor_profiles) 및 모드별 제어 알고리즘 완전 유지
 * - 제어 상태 변수(MotorState) 구조체화로 독립적 제어 보장
 * ============================================================================
 */

#include <Arduino.h>
#include <mbed.h>
#include <math.h>

#include <PortentaEthernet.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// [21차] SD BIN 로깅: 캐리어(Hat Carrier 등)의 SDMMC 슬롯 + FATFS + SRAM 링버퍼
#include <SDMMCBlockDevice.h>
#include <FATFileSystem.h>
#include <sys/stat.h>

#include "src/MD80Comm.h"

// ============================================================================
// [UDP & Network Configuration]
// ============================================================================
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(172, 26, 178, 50); // RIGHT ARM (IP_RIGHT_ARM)
unsigned int localPort = 9999;
EthernetUDP Udp;

IPAddress jetson_ip;
uint16_t jetson_port = 0;
bool jetson_connected = false;

// ============================================================================
// [UDP Structures]
// ============================================================================
#pragma pack(push, 1)
enum ControlMode : uint8_t { 
    CMD_IDLE = 0, CMD_AROM = 1, CMD_PROM = 2, CMD_CPM = 3, CMD_ISOM = 4, CMD_ISOT = 5, CMD_FIX_POS = 6 
};

struct JointCmd {
    uint8_t mode;
    uint32_t command_id;
    float target_pos_min; 
    float target_pos_max; 
    float speed;         
    float resistance;    
};

struct JetsonToM7_UDP {
    uint8_t robot_state;
    uint32_t seq;
    JointCmd joints[4];
};

struct JointStateMsg {
    float pos;
    float vel;
    float tau_meas;
    float tau_cmd;
    uint8_t online;
    uint8_t isom_phase;
    uint8_t isom_rep;
    uint8_t isom_total_reps;
    uint32_t isom_remaining_ms;
};

struct M7ToJetson_UDP {
    uint8_t robot_state;
    uint32_t seq_echo;
    uint32_t m7_rx_to_tx_us;
    uint32_t fdcan_rtt_us;
    JointStateMsg joints[4];
};
#pragma pack(pop)

static_assert(sizeof(JetsonToM7_UDP) == 89, "JetsonToM7_UDP layout mismatch");
static_assert(sizeof(M7ToJetson_UDP) == 109, "M7ToJetson_UDP layout mismatch");

uint32_t last_seq_echo = 0;

// ============================================================================
// [Watchdog Shared Memory (SRAM4)]
// ============================================================================
struct WatchdogData {
    volatile uint32_t m7_heartbeat;
    volatile uint8_t motor_comm_failed; // 1 = 모터 통신 완전히 두절
    volatile uint8_t e_stop_triggered;
};
volatile WatchdogData* wd_data = (volatile WatchdogData*)0x38001000;


// ============================================================================
// [Motor Profiles Configuration]
// ============================================================================
// [Patient & Dumbbell Configuration (Winter 2009)]
static constexpr float HUMAN_M_KG = 55.0f;    // 환자 체중 [kg]
static constexpr float HUMAN_H_M  = 1.65f;    // 환자 신장 [m]

// Winter segment lengths
static constexpr float L_UPPERARM = 0.186f * HUMAN_H_M;
static constexpr float L_FOREARM  = 0.146f * HUMAN_H_M;
static constexpr float L_HAND     = 0.108f * HUMAN_H_M;

// Dumbbell lever arm distances from rotation axis
// 1. 어깨 (어깨 중심축 ~ 손)
static constexpr float DUMBBELL_L_SHOULDER = L_UPPERARM + L_FOREARM + 0.3f * L_HAND;
// 2. 팔꿈치 (팔꿈치 중심축 ~ 손)
static constexpr float DUMBBELL_L_ELBOW = L_FOREARM + 0.3f * L_HAND;
// 3. 손목 (손목 중심축 ~ 손)
static constexpr float DUMBBELL_L_WRIST = 0.3f * L_HAND;
// [38차] 4. 어깨 내외회전 (회전축 = 상완 장축 ~ 손). Ro 시작자세는 상완 수평 + 팔꿈치 90° 굴곡
//   이므로 전완이 축에 수직하게 매달린다 → 축에서 손까지의 수직거리 = 전완 + 손 (팔꿈치와 동일).
static constexpr float DUMBBELL_L_SHOULDER_RO = L_FOREARM + 0.3f * L_HAND;

struct MotorProfile {
    uint32_t id;
    String name;
    float dob_j;
    float dob_b;
    float cpm_kp;
    float cpm_kd;
    float gc_m;
    float gc_l;
    float max_torque;
    float dumbbell_l; // Isotonic 모드용 가상 덤벨 레버 암 길이
};

static const int JOINT_COUNT = 4;

// 4축 프로필 정의 (Joint 3 은 신형 모델에서 삭제 — 미연결)
// [25차] 2026-08 리그 리디자인: CAN ID 100/20/10 → 110/130/120 (can_id_scanner 실측 +
//   손 움직임으로 ID↔관절 매핑 확인). 구 손목 ID 130 이 회전관절로 재사용됨에 주의.
MotorProfile motor_profiles[JOINT_COUNT] = {
    // 1. AK80-64 (ID: 110) - 어깨 굴곡/신전
    // rShoulderEF 링크 중력보상 데이터는 모터 변경과 관계없이 유지합니다.
    // [31차] max_torque 30→48 (AK80-64 rated). 착용 CPM(09-07 132457) 에서 −50° 요구토크 15+ Nm → 클램프 포화·stall.
    // [31차] cpm_kp/kd 30/2 → 50/5 (FIX·자세전환 전용 게인; PROM_PD 검증값과 동일). Ro 시작자세 EF -90° 전환에서
    //   순수 PD 30 이 중력+마찰(6.8 Nm)과 -79° 에서 비겨 posture 미완료(09-07 135326). ζ 0.27→0.53.
    // [41차] cpm_kp/kd 50/5 → 70/6.0 (run_fix_pos 전용 — CPM/PROM 은 PROM_PD_* 를 쓴다).
    //   계기: 10-07 착용 AROM(074645)에서 Ro 시작자세 전환이 **−77.8° 에서 15 초간 정체**
    //   (지령 15.25 Nm 고정, 이동 0.00°, 오차 12.2° 로 허용오차 5.7° 밖). 전체 전환에 31.95 s.
    //   원인은 run_fix_pos 가 순수 PD + 중력보상(DOB 없음)인데 **사람 팔 무게(~7 Nm)가 모델에
    //   없어** PD 가 전부 떠안고 거기서 비긴 것. #31 의 30→50 과 같은 계열(그때는 −79°).
    //   감쇠비 유지: ζ = kd/(2√(kp·J)), J = ST_J[0] = 0.452 → 0.526(50/5) ≈ 0.533(70/6.0).
    //   정체 지점 기준 PD 몫 10.6 → 14.9 Nm. [41차] FIX_GC_ML[0] 보정분 4.20 과 합쳐 +8.5 Nm.
    { 110, "AK80-64", 0.010f, 0.050f, 70.0f, 6.0f, 0.9243f, 0.095f, 48.0f, DUMBBELL_L_SHOULDER },
    // 2. AK60-6 (ID: 130) - 어깨 내회전/외회전
    // [24차] max_torque 6→3 (출력축; AK60-6 rated = 0625 하드천장 0x0112 과 동일. sector 4:1
    //   이라 관절토크 최대 12 Nm — posture 전환은 이 전체, 평시 FIX 는 40% 캡)
    //   [38차] dumbbell_l: 어깨 전체(0.601) → 내외회전 전용(0.294). 가상 덤벨은 전완 끝의 손에
    //   걸리고, 레버는 회전축(상완 장축)에서 손까지의 수직거리다 — 어깨 EF 의 레버가 아니다.
    { 130, "AK60-6(ShoulderRo)", 0.00285f, 0.01f, 5.0f, 0.5f, 0.0f, 0.0f, 3.0f, DUMBBELL_L_SHOULDER_RO },
    // 3. AK70-10 (ID: 120) - 팔꿈치 굴곡/신전
    // [30차] gc_m·gc_l = 0.0254 kg·m (신형 MJCF link4+cuff, mujoco qfrc_bias: τ_g = 0.249·sin q, 오차 <0.001).
    //   j2 는 DOB off 라 τ_g 가 직접 작용 (어깨의 이중보상 문제 없음). 굴곡 중 중력이 slip 을 밀던 −0.22 Nm 편향 제거.
    // [32차] 손목 구조물(AK60-6 포함) 장착 링크: gc_m·gc_l 0.0254→0.1643 kg·m (τ_g 1.61 Nm @ −90°+φ), 위상 GC_PHASE_RAD[2] 참조.
    { 120, "AK70-10", 0.004f, 0.011f, 15.0f, 1.0f, 0.600f, 0.2738f, 8.3f, DUMBBELL_L_ELBOW },   // [31차] max_torque 20→8.3 (rated)
    // 4. (미사용) 손목 — 신형 모델에서 관절 자체가 삭제. ID 140 은 버스에 없는 placeholder
    //    (구 값 130 은 이제 회전관절 ID 라 충돌 — 절대 되돌리지 말 것)
    // [35차] 4번째 관절 rWrist 활성화 (2026-09-22): AK60-6 **direct-drive**(외부 기어 없음, 작은 손잡이 링크) CAN ID 20.
    //   MD80 임피던스 모드로 운용(elbow 와 동일 경로). max_torque 3.0 = AK60-6 rated. gc 0 (손잡이 질량 미상 — 첫 런 로그로 식별).
    { 20,  "AK60-6(Wrist)", 0.00285f, 0.01f, 5.0f, 0.5f, 0.0f, 0.0f, 3.0f, DUMBBELL_L_WRIST }
};

// ============================================================================
// [Global Configuration] 
// ============================================================================
// 연결된 3개 모터를 매 제어 주기마다 순차 폴링하는 안정 우선 구성
// [37차] 연결 관절은 **enable 시 버스 스캔**으로 결정한다 (하드코딩 폐지).
//   전제 = ID↔관절 규약 고정: 110=j0 rShoulderEF / 130=j1 rShoulderRo / 120=j2 rElbow /
//   20=j3 rWrist. 다른 ID 의 모터를 다른 관절에 다는 운용은 지원하지 않는다 (CAN ID 는
//   기구 정보를 담지 않으므로, 규약이 유일한 ID→관절 근거다).
//   ★스캔은 Portenta 부팅 후 최초 enable 1 회만 — boot zero 와 같은 래치 의미. 운전 중
//   드롭은 종전대로 E-stop 경로다. 여기서 재스캔하면 통신이 끊긴 관절이 E-stop 대신
//   조용히 제외되어 힘이 빠진 채 남는다 (어깨는 팔 낙하 경로).
static bool MOTOR_CONNECTED[JOINT_COUNT] = {false, false, false, false};
static int  CONNECTED_MOTOR_COUNT = 0;      // 진단 출력용 (판정은 MOTOR_CONNECTED 로만)
static bool motor_scan_done = false;
// [24차] rShoulderRo(j1) 외부 4:1 sector 기어 (0625_cpm_shoulder_yaw 관례).
//   MD80 기어비 레지스터(0x0017)는 1/6 유지 → 드라이브가 보고하는 각도 = 6:1 "출력축" 프레임.
//   j1 의 내부 제어·guard·ST_*·게인·SD/UDP 로그는 전부 출력축 프레임 (관절각 = 출력축/4,
//   관절토크 = 출력축토크×4). 식별값과 0625 검증 게인이 출력축 프레임이라 내부는 무변환이
//   안전하고, UDP 경계에서만 환산: 수신 명령(관절 rad)×4, 송신 상태 pos·vel /4, 토크 ×4.
//   ★ 운용 [26차 갱신]: guard 기준 = M7 최초 enable(런치) 시점 자세 (enable 경로에서 0x008C
//   영점). ros2 launch 직전에 회전관절을 기준자세(내회전 0°)에 둘 것 — 모터 전원 인가 자세는
//   무관. 어긋나면 guard 가 하드스톱(신형 리그 실측 관절 -32.3°/+76.5°, 2026-08-10)과
//   어긋남 → stall 감지가 최후 방어선.
static const float JOINT_GEAR[JOINT_COUNT] = {1.0f, 4.0f, 1.0f, 1.0f};
static const float LOOP_RATE_HZ = 500.0f;
static const float DOB_Ts = 1.0f / LOOP_RATE_HZ;
static const float POSTURE_SPEED_RAD_S = 0.05f;
// [14차] posture "전환"만 3배 속도 — 0.05 rad/s(2.9°/s)는 관절이 몇 도만 벗어나 있어도
//   시작까지 수 초 침묵. 0.05 는 cpm/isometric 기본속도 겸용이라 그대로 두고 전환만 분리.
static const float POSTURE_TRANSITION_SPEED_RAD_S = 0.15f;
// [jehyeon 2026-07-18] 0.02 → 0.10: run_fix_pos 는 마찰보상 없는 순수 PD 라 저마찰 가정.
//   AK80-64 는 실측 정지마찰 2.3~4.3 Nm >> PD@0.03rad(≈1 Nm) 로 1.1° 안에 못 들어와
//   posture 가 영원히 미완료 → PROM 진입 불가였음. 5.7° 는 PROM/시연 용도에 무해.
//   (근본 해결은 fix/posture 에 마찰 FF+DOB 이식 — Stribeck 포팅 때 함께)
static const float POSTURE_POSITION_TOL_RAD = 0.10f;
static const float POSTURE_VELOCITY_TOL_RAD_S = 0.05f;
static const float FIX_TORQUE_RATIO = 1.0f;     // [31차] 0.4→1.0: FIX 홀드 = 프로필 정격 그대로 (j0 48 / j1 3 / j2 8.3)
static const float FIX_TORQUE_MIN_NM = 1.5f;
static const float FIX_TORQUE_MAX_NM = 48.0f;   // [31차] 5→48: 캡 무효화 (정격이 상한)
// [31차] FIX/자세전환 전용 중력 M·L [kg·m] — run_fix_pos 는 DOB 가 없어 정확한 중력값이 맞음 (CPM/PROM 의
//   프로필 gc 는 DOB 이중보상 때문에 구값 유지). j0 0.4836 = 신형 MJCF 합성값(τ_g 4.74 Nm @ -90°),
//   j2 0.0254 = 프로필과 동일. 0 이면 프로필 gc_m·gc_l 사용.
// [37차] const 해제 — 손목 모듈 유무에 따라 applyModuleConfig() 가 j2 값을 바꾼다.
// [41차] ★j0 0.4836 → 0.9823 (2.03배). 구값은 09-07 에 MJCF(ak80-64_upper_limb_exo.xml)에서 뽑은
//   것인데 그 모델은 **3자유도라 손목 모듈이 없다**. 손목(AK60-6 + 손잡이)은 09-22 에 물리적으로
//   추가되었으므로 로봇이 그만큼 무거워졌고, FIX·자세전환은 DOB 가 없어 이 값이 직접 작용한다
//   → 어깨가 −90° 까지 못 가고 처진다.
//   재식별: 10-06 j0 PROM 로그(M7LOG_002_20261006_042916, 47.5 s, 관절 −66.1~0°, 정속 17,814 샘플)
//   를 각도 12빈으로 나눠 전진·후진 평균 → 중력. τ_g = 9.6326·sin(q +1.3°) +0.69 Nm, **rms 0.062**.
//   위상 +1.3° ≈ 0 (굴곡관절이므로 q=0 에서 중력 0 — 모델과 일치). 마찰 2.60~3.10 Nm.
//   교차검증: MJCF 전체(손목 없음) 0.4838 + 손목 모듈 추정 ~0.44 kg·m ≈ 0.92 로 실측 0.9823 과 부합.
//   τ_g @ −90°: 4.74 → **8.94 Nm** (부족분 4.20 Nm 해소).
static float FIX_GC_ML[JOINT_COUNT] = { 0.9823f, 0.0f, 0.1643f, 0.0f };   // [32차] j2 0.0254→0.1643 (손목 구조물 포함 링크, 09-14 식별)
// [32차] 중력 위상 [rad]: τ_g = M·L·g·sin(q + φ). COM 이 링크 축에서 벗어난 경우(손목 모터 오프셋). 모든 τ_g 계산에 공통.
//   j2 −11° : 09-14 CPM 1단(121938) 굴곡/신전 평균 토크 LS 피팅 (1.581·sin q − 0.308·cos q, 잔차 0.04 Nm; sin 단독 0.31).
static float GC_PHASE_RAD[JOINT_COUNT] = { 0.0f, 0.0f, -11.0f * M_PI / 180.0f, 0.0f };
// [37차] 손목 구조물을 뗀 elbow 링크의 중력 M·L [kg·m] — #30 신형 MJCF link4+cuff 식별값
//   (mujoco qfrc_bias, τ_g = 0.249·sin q). 현재 0.1643 은 손목 모듈 포함값(#32).
static const float ELBOW_GC_ML_NO_WRIST = 0.0254f;
static const float PROM_DEFAULT_SPEED_RAD_S = 0.05f;
static const float PROM_KP_VEL = 1.5f;
static const float PROM_KI_VEL = 10.0f;
static const float PROM_INTEGRAL_LIMIT = 10.0f;
// [jehyeon 2026-07-18] PROM 관절별 안전층 (0716_prom_elbow 검증 이식).
//   guard 프레임 = 드라이브 절대각. [26차부터] 최초 enable 시 0x008C 영점 → "런치 직전
//   기준자세 = 0" (모터 전원 인가 자세는 무관해짐). PROM 은 시작자세(0) 이동 후
//   실행되므로 사실상 시작자세 기준 상대각과 동일.
// [25차] 2026-08 리그 리디자인 — guard = 신형 MJCF(ak80-64_upper_limb_exo.xml) ROM 기준
//   (사용자 지침 "trust joint limit in MJCF"). 2026-08-10 position_encoder_test 실측 하드엔드:
//   j0 -180°(가동 끝; 스토퍼는 -185° — 28차 정정), j1 출력축 -129°/+306°(관절 -32.3/+76.5),
//   j2 -124°(MJCF -120 이 자체로 4° 마진).
static const float PROM_GUARD_MIN_RAD[JOINT_COUNT] = {
    -180.0f * M_PI / 180.0f,   // 0 rShoulderEF: MJCF -180..0° 그대로. [28차] -175→-180: 물리
                               //   스토퍼는 가동범위 바깥 5°(-185°)라 마진은 스토퍼가 담당.
                               //   09-07 실기: -180.00° 도달, 홀드 0.19 Nm, stall 없음(미접촉)
    -1.0472f,                  // 1 rShoulderRo: ★출력축 rad = 관절 -15° (MJCF). 실측 하드엔드
                               //   관절 -32.3° (출력 -129°) — external 방향은 backstop 용
    -120.0f * M_PI / 180.0f,   // 2 rElbow: MJCF -120° (굴곡 하드엔드 실측 -124° → 4° 마진)
    -60.0f  * M_PI / 180.0f    // 3 rWrist: [35차] 기구 ROM 0..-90° 이나 착용 시 -90 은 과도(09-22 사용자 체감) → guard -60 (사람 안전 한계)
};
static const float PROM_GUARD_MAX_RAD[JOINT_COUNT] = {
     5.0f   * M_PI / 180.0f,   // 0 rShoulderEF: 신전 하드엔드 0° + 시작자세 슬랙 (elbow 관례 동일)
     5.2360f,                  // 1 rShoulderRo: ★출력축 rad = 관절 +75° (MJCF. 실측 하드엔드
                               //   관절 +76.5°/출력 +306° — 마진 1.5° 박함, 운용 CPM 은 0→+45° 권장)
     5.0f   * M_PI / 180.0f,   // 2 rElbow: 신전 하드엔드 0° + 시작자세 슬랙
      5.0f  * M_PI / 180.0f    // 3 rWrist: 신전 끝 0° + 시작자세 슬랙 (EF/elbow 관례)
};
// PROM 전용 토크 클램프 (프로필 max_torque 이전 적용) — 사람 팔/하드스톱을 미는 모드라 보수적.
//   elbow 4.0 = 0716 검증값 (사람 팔 CPM 실측 max 2.97 + 여유). 어깨 EF 는 중력 몫으로 10.
//   [24차] j1 = 1.0 (출력축 — 0625 torque_limit 검증값; sector 4:1 이라 관절토크 ~4 Nm).
// [30차] j0 10→15: 신형 리그 어깨는 −80° 부근에서 중력 4.7+마찰 3.6+DOB 몫 ≈ 10~11.6 Nm 필요
//   (09-07 PROM 스윕 2런 모두 −79° 에서 −10.00 클램프 포화 → stall 홀드; CPM 은 15 로 피크 11.6 실측 통과).
// [31차] j0 15→48 (rated), j2 4→8.3 (rated). 사람 팔(어깨 ~6 Nm, 전완 ~2 Nm) + 외골격 + 마찰을 고려.
static const float PROM_TORQUE_MAX_NM[JOINT_COUNT] = { 48.0f, 3.0f, 8.3f, 3.0f };   // [35차] j3 3.0 (AK60-6 rated)   // 정격: j0 48 / j1 출력축 3(관절 12) / j2 8.3
// stall(하드스톱/잼/과저항) 감지: 스윕 지령 대비 실속도가 낮은 상태가 지속되면 홀드 전환.
//   1500 ms — PROM PI(Ki=10) 기동 시 마찰 극복까지 ~1 s 걸리는 것을 감안한 여유.
// [jehyeon 2026-07-18 v2] PROM 을 0716_prom_elbow 구조(기준위치 램프 + PD + Stribeck DOB)로
//   전환하면서 속도서보용 stall 상수(비율/1.5s)는 0716 의 위치오차 기준으로 교체.
//   (PROM_KP_VEL/KI/INTEGRAL_LIMIT 는 구 속도서보 상수 — 미사용 보존)
static const float PROM_ACC_RAD_S2   = 0.1f;            // 가감속 램프 [rad/s²] (0716 ACC_PROM)
static const float PROM_TF           = 1.0f / 200.0f;   // Stribeck DOB 1차 LPF (0611/0716 lpf_f=200)
static const float PROM_VEL_CUTOFF   = 1.0f;            // |필터속도| 컷오프 [rad/s] (0716 안전 2)
// [30차] 관절별 컷오프. elbow 1.0→1.5: 신형 리그 elbow 는 J 0.007 이라 정지마찰 breakaway 한 번이
//   40~60 ms 동안 60~70°/s(≈1.1~1.2 rad/s)를 찍고 2° 이동 후 PD 가 제동 — 폭주가 아닌데 1.0 에 걸림
//   (09-07 런 115449: 85 s 왕복 중 slip 4015회 중 1회가 68°/s → 홀드). stall(8.6°/250 ms)·토크
//   클램프 8.3 Nm 은 그대로. ★환자 세션 전 이 값의 적정성은 별도 판단 (벤치 특성 기준 값).
static const float PROM_VEL_CUTOFF_J[JOINT_COUNT] = { 1.0f, 2.0f, 1.5f, 1.5f };   // [35차] j3 1.5 (elbow 와 동일)   // [31차] j1 1.0→1.5→2.0 (출력축; 관절 29°/s). Ro CPM 0.838 단에서 sector 치합 속도리플(0.13↔1.6, 25 Hz)이 1.49 → 컷오프. 리플은 기어 상태 지표 — 윤활/재조립 후 재평가
// [24차] stall 한계를 관절별로: j1 은 0625 검증값 0.30(출력축 rad ≈ 관절 4.3°) — 클램프 1.0 Nm
//   에선 Kp=5 로 오차 0.2 rad 에서 토크 포화라 0.15 로는 정상 운전 중 오탐. 나머지는 기존 0.15.
// [34차] j2 0.15→0.06 rad(3.4°): 임피던스 kp 200 에서는 오차 2.4° 이상이면 이미 maxTorque(8.3) 포화 → 지속 오차 3.4° = "사람/장애물을
//   정격 근처 토크로 밀고 있음". 09-15 착용 CPM 에서 사용자 ROM 끝(−116°)에 −120 참조가 4.4° 오차·6.85 Nm 으로 10 s 동안 눌렸으나
//   8.6° 기준에는 안 걸림. 어깨·Ro 는 외부 PD 라 기존 값 유지.
static const float PROM_STALL_ERR_RAD[JOINT_COUNT] = { 0.15f, 0.30f, 0.06f, 0.10f };   // [35차] j3 0.10 (kp50: 5.7° 오차 = 5 Nm > rated 3 → 포화 지속 = 사람/장애물)
static const uint32_t PROM_STALL_ERR_MS = 250;          //   지속 시간
// PROM 전용 PD 게인 (실측 검증값: AK80-64=ak80-64_cpm_moving, elbow=0611/0716, AK60-6=0625)
// [29차] elbow(j2) 6/0.7 → 50/2.0: 신형 리그 CPM 속도 스윕(09-07, DOB off 순수 PD)에서 Kp 6 은
//   마찰(~0.75~0.95 Nm)과 비겨 -120° 미도달(-113°), 저속 stick-slip(2.9°/s 에서 56% 정지),
//   max|e| 8.5°(stall 8.6° 직전). 개인연구 25/1.2 검증 위에서 과감히 상향 — J≈0.012 기준
//   ωn≈10 Hz, ζ≈1.3, stall 한계 8.6° 에서 토크 7.5 Nm(클램프 8.3 안). 발진 시 25/1.2 로 롤백.
//   [30차] elbow 50/2.0 → 100/2.5: Kp50+FF0.8 런(113245)에서 stick-slip 잔존(멈춤 26%, slip 피크 56°/s
//   → -96° 컷오프). slip 에너지 ∝ Fs²/Kp 라 Kp 2배로 절반. ωn≈19 Hz, ζ≈1.5. (isometric 은 200 검증)
//   → 100/2.5 는 실기에서 모터 정지(E-stop/발진) — 50/2.0 으로 원복. Kp 상향은 한계 확인됨.
//   → 디더 런(114722): stick-slip 은 해결(RMSE 0.22°, slip 피크 p90 11°/s)됐으나 25 Hz PD 루프 발진
//   (3주기 만에 ±20→±80°/s 성장)으로 컷오프. 원인 = Kd 가 32 Hz 필터속도(지연 ~40°)+2 tick 지연에
//   걸려 고주파에서 음의 감쇠. elbow 35/1.2 로 하향 (ζ≈1.2 명목, 25 Hz 이득 여유 확보).
//   [32차] elbow 35/1.2 → 80/3.0: 손목 구조물 장착으로 J ≈0.007→~0.04 (5배). ωn = √(80/0.04) ≈ 7 Hz(구링크 Kp35 의
//   10 Hz 보다 낮음 → 발진 여유 ↑), ζ ≈ 0.85. 정지 감김 Fs/Kp 0.65/80 = 0.5°. 발진 시 50/2.0 으로 롤백.
//   → 80/3.0 은 신링크에서 시작 즉시 발진(09-14) — 50/2.0 으로 원복.
static const float PROM_PD_KP[JOINT_COUNT] = { 50.0f, 5.0f, 50.0f, 5.0f };
static const float PROM_PD_KD[JOINT_COUNT] = { 5.0f, 0.15f, 2.0f, 0.5f };
// 관절별 Stribeck 마찰/관성 (★PROM 경로 전용 — 타 모드의 dob_j/dob_b 는 무수정)
//   0: AK80-64 식별 2026-07-17 (motor_configuration_test_ak80-64, vs=0.10 은 실측 — 관례 0.5 아님)
//   1: AK60-6 0625 bare 6:1 출력축 값 — sector 기어 마찰 미포함이라 과소보상(안전 방향)
//   2: AK70-10 0611 확정값 / 3: 손목 미연결 placeholder (fc=fs=0 → 점성만)
// [39차] ★j1 재식별 (조립 상태). 09-07 CPM 10단 중 1~7단(135914~141141) 7개 로그, 정속 구간을
//   각도 29빈으로 나눠 전진/후진 평균 → 중력, 차이/2 → 마찰 (n=202). #32 의 elbow 방식과 동일.
//   - 마찰(출력축)은 속도에 거의 무관: 0.3203(0.05~0.15 rad/s) → 0.3078(0.6~0.8) → 점성 ≈ 0,
//     완만한 Stribeck 하강만 있음. 각도 의존은 관절 1.19(−15~0°) → 1.45 Nm(+60~75°).
//   - ST_FC 는 **ROM 최솟값 0.297** 을 쓴다: AROM 보상 = 0.7·ST_FC = 0.208 < 실제 마찰(≥0.297)
//     이 전 구간에서 성립 → 과보상에 의한 자가구동이 구조적으로 불가능(안전 방향).
//   - 종전 값은 0625 bare 모터 식별값으로 sector 기어 마찰이 빠져 있었다(실측의 1/2.8).
//   ★ST_FC/FS/FV 는 CPM·PROM·isotonic 의 DOB 식에서 `disturbance + tau_st` 로 **정확히 상쇄**
//     되므로(run_cpm/run_prom 참조) 검증된 그 모드들의 출력은 바뀌지 않는다. AROM 에서만 직접 쓰인다.
static const float ST_J [JOINT_COUNT] = { 0.452f,  0.0024f, 0.0103f, 0.0f   };
static const float ST_FV[JOINT_COUNT] = { 0.12f,   0.0f,    0.0096f, 0.01f  };   // [39차] j1 0.007→0 (점성 성분 관측 안 됨)
static const float ST_FC[JOINT_COUNT] = { 2.1f,    0.297f,  0.25f,   0.0f   };   // [39차] j1 0.105(bare)→0.297(조립, ROM 최솟값)
static const float ST_FS[JOINT_COUNT] = { 3.59f,   0.330f,  0.578f,  0.0f   };   // [39차] j1 0.15→0.33
static const float ST_VS[JOINT_COUNT] = { 0.10f,   0.50f,   0.50f,   0.50f  };
// [27차] 신형 리그(2026-08) 대응: 관절별 Stribeck DOB+마찰 FF 스케일 (1=정상, 0=순수 PD).
//   elbow 는 구형 하완(336 g) 식별값이 신형(링크+커프 ~104 g, 관절마찰↑)과 달라 CPM/PROM
//   에서 ~15 Hz 한계사이클 → 과속컷오프 발동이 재현됨 (08-10 로그 2건: 토크 ±1 Nm 채터,
//   dist 0→1.1 Nm 톱니). 재식별 전까지 j2 는 DOB/FF OFF = 순수 PD (같은 게인의 FIX 홀드가
//   같은 날 로그에서 안정 검증됨). 어깨 둘은 08-10 실기 통과라 유지. disturbance 는 스케일과
//   무관하게 계속 로깅 → 재식별 참고 데이터.
static const float ST_DOB_SCALE[JOINT_COUNT] = { 1.0f, 1.0f, 0.0f, 1.0f };
// [30차] 참조속도 기반 마찰 피드포워드 (CPM/PROM, 관절별 스케일 — 0 = off).
//   elbow: 신형 리그 DOB 는 J 가 작아(0.007) Q=200 Hz 미분항이 정지마찰 경계에서 양의 되먹임
//   → 09-07 실기 28 ms 발산(로그 112712). 대신 측정속도가 아닌 v_ref 로만 쿨롱+점성을 미리
//   밀어 stick-slip 을 줄인다 — 되먹임 루프 없음, |τ_ff| ≤ scale·(Fc+Fv·v). Fs 봉우리는 미포함
//   (출발 과잉 어시스트 방지, Kp 50 이 담당). 값: 09-07 속도 스윕 재식별 Fc 0.29 / Fv 0.027.
static const float FF_REF_SCALE[JOINT_COUNT] = { 0.0f, 0.0f, 0.5f,   0.0f };   // 0.8→0.5: slip 에너지 기여 감소 (113245)
static const float FF_REF_FC   [JOINT_COUNT] = { 0.0f, 0.0f, 0.20f,  0.0f };   // [Nm]  [32차] 0.29→0.20 (신링크 09-14 굴곡/신전 반차 0.17~0.24)
static const float FF_REF_FV   [JOINT_COUNT] = { 0.0f, 0.0f, 0.027f, 0.0f };   // [Nm·s/rad]
// [30차] 디더(dither): 관절이 완전히 정지마찰에 "붙는" 상태를 막기 위한 소진폭 고주파 토크.
//   저관성·고정지마찰(elbow: J 0.007, Fs 0.65/Fc 0.29) 조합의 stick-slip 완화 — Kp 상향(100 발진),
//   DOB(폭주), FF(slip 에너지↑)가 모두 실패한 뒤의 처방(09-07). 유계 개루프 신호라 발진 무관.
//   이동 중(|v_ref|>0)에만 인가, 끝점 홀드에서는 0. 진폭 0 = off.
static const float DITHER_A [JOINT_COUNT] = { 0.0f, 0.0f, 0.0f,  0.0f };   // [33차] j2 0.5→0: 임피던스 모드에서 불필요   // [Nm]  0.25→0.35→0.50 (Fs 0.65 의 77% 상쇄; 상한 ≈ Fs)
static const float DITHER_HZ            = 50.0f;                          // 500 Hz 루프에서 10 샘플/주기
// ============================================================================
// [33차] MD80 드라이버 임피던스 모드 (관절별). 드라이버 내부 루프가 τ = kp·(q_ref−q) + kd·(v_ref−v) + τ_ff 를
//   계산하므로 500 Hz 통신·속도필터 지연이 루프 밖으로 나감 → 외부 PD 로 불가능했던 고강성이 안정 (09-07/14: elbow
//   외부 PD Kp 80~100 에서 발진, stick-slip 잔존). IMP_JOINT=true 관절은 enable 시 IMPEDANCE 모드로 켜고, 참조를
//   쓰는 모드(CPM/PROM/FIX/ISOM/ISOT-MOVING)에서는 (q_ref, v_ref, τ_ff=중력+마찰FF) 를 보내고, 토크 전용 모드
//   (ISOT-ACTIVE/AROM)에서는 kp=kd=0 으로 바꿔 τ_ff 로 우리 토크를 그대로 보낸다 (전환은 모드 바뀔 때 1회, blocking).
//   stall/guard/컷오프는 그대로(참조·측정 기준). 드라이버 maxTorque(0x0112) = 프로필 max_torque 로 설정.
//   Portenta 측 PD(PROM_PD_*)·DOB·디더는 이 관절에서 사용되지 않음(로그 tau_cmd = τ_ff).
// ============================================================================
static const bool  IMP_JOINT[JOINT_COUNT] = { false, false, true,  true  };   // [35차] j3 도 임피던스 (저관성 direct-drive → 외부 PD 는 elbow 와 같은 stick-slip/발진 경로)
static const float IMP_KP   [JOINT_COUNT] = { 0.0f,  0.0f,  200.0f, 50.0f };   // [35차] j3 50 초기값 (rated 3 Nm: 오차 3.4° 에 포화). 첫 런 후 100 검토   // [Nm/rad] 드라이버 임피던스 강성 (출력축)  [33차] 100→200 (kd5 검증 후 가감속 오차 절감용)
static const float IMP_KD   [JOINT_COUNT] = { 0.0f,  0.0f,  7.0f,   1.0f };   // [35차] j3 1.0 초기값
// [35차] CAN 타이밍 (관절별, µs). AK60-6 개체(130 실측 09-07: batch read 0%, write 앞 ≥75 µs 필요)와 같은 성향의 ID 20 대응.
//   READ_GAP: 위치 read 요청 앞 정적 간격. IMP_WRITE_GAP: 임피던스 3프레임 앞·사이 간격 (j2 60 → 기존과 동일).
//   ★ID 20 이 그래도 async timeout 이면 IMP_JOINT[3]=false 로 토크 모드 폴백(외부 PD 5/0.5) — 통신부터 확보.
static const uint32_t READ_GAP_US     [JOINT_COUNT] = { 0, 0, 0, 150 };
// [37차] 토크 읽기 앞 간격 — 토크 요청이 "직전 위치 읽기와 같은 드라이브" 를 겨냥할 때만
//   삽입한다. 드라이브는 자기 응답 프레임 직후 ~50 us 안에 오는 프레임을 떨어뜨린다.
//   값 150 = j3 READ_GAP_US 검증값과 동일. 4틱(3모터는 3틱)에 1회만 들어가므로 평균 +37 us.
static const uint32_t TAU_READ_GAP_US = 150;
static const uint32_t IMP_WRITE_GAP_US[JOINT_COUNT] = { 60, 60, 60, 150 };
//   (헬퍼 fricFFRef 는 MotorState 정의 뒤, stribeckFriction 옆에 정의 — Arduino 프로토타입 삽입 위치 문제)
// CPM 전용 토크 클램프 (rated 기준 보수적: AK80-64 는 중력·부하 여유로 15)
//   [24차] j1 = 1.0 (0625 CPM 검증 클램프 — LOG328~330 이 값으로 관절 0.05° 추종 확인)
// [31차] j0 15→48 (rated) — 착용 CPM 포화 해소.
static const float CPM_TORQUE_MAX_NM[JOINT_COUNT] = { 48.0f, 3.0f, 8.3f, 3.0f };
// [36차] isotonic 소프트 월 (2026-09-29): guard 밖에서 토크를 0 으로 끊던 동작을 "guard 로 되돌리는
//   스프링+댐퍼"로 교체. 구 동작의 문제 — ① 경계에서 단차(팔꿈치 1 kg 1.3 Nm, 5 kg 8.3 Nm; 손목 1 kg
//   0.45 Nm) ② 덤벨·중력보상 토크는 둘 다 0° 쪽으로 되돌리는 방향이라, 끊으면 오히려 가동범위 밖으로
//   가속됨 ③ 중력보상까지 사라져 팔이 낙하. 새 동작: 덤벨·중력 항을 그대로 유지한 채(경계에서 연속)
//   초과분에 비례하는 복원 토크를 더한다. 포화 거리 = TAU_MAX/KP: 어깨 14°, 팔꿈치 12°, 손목 5.7°.
// [38차] j1 신설 — 값은 **출력축 프레임**이다. 관절 프레임 환산: K_joint = 16·K_out,
//   D_joint = 16·D_out, tau_joint = 4·tau_out (sector 4:1).
// [38차-b] 1차 값(K_out 2.5 = 관절 40, 상한 1.0 = 관절 4)은 실기에서 "벽처럼 멈추기만 하고
//   밀어내지 않음"(10-07 061648). 로그 분석: 침투 1.0~1.78° 로 정지 기능은 정상이고 상한
//   포화는 0% 였으나, 관절 −15.37° 에서 복원 2.5 Nm 를 걸고도 0.24 s 동안 속도 ≈ 0 —
//   구간의 84% 가 정지. **sector 마찰(관절 ≥2.5 Nm)이 복원 토크를 전부 흡수**한 것이라
//   상한이 아니라 **기울기**를 올려야 한다.
//   → K_out 2.5→7.5 (관절 120), D_out 0.125→0.25 (관절 4.0, K 3배에 맞춰 감쇠 보강),
//     상한 1.0→2.0 (관절 8.0 = 팔꿈치 8.3 과 동급, 출력축 rated 3.0 의 67%).
//   포화 깊이 = 2.0/7.5 = 0.267 rad(출력축) = 관절 3.8°. 굴곡 쪽 하드엔드까지 17.3° 여유.
static const float ISOT_WALL_KP     [JOINT_COUNT] = { 60.0f, 7.5f,  40.0f, 30.0f };   // [Nm/rad]
static const float ISOT_WALL_KD     [JOINT_COUNT] = {  4.0f, 0.25f,  2.0f,  1.0f };   // [Nm·s/rad]
static const float ISOT_WALL_TAU_MAX[JOINT_COUNT] = { 15.0f, 2.0f,   8.3f,  3.0f };   // [Nm] 사람을 미는 벽이라 rated 보다 보수적(어깨·회전관절) 
// [jehyeon v2] AROM (0522_ad_imp_arom 검증 파라미터) — admittance 가상 동역학 + impedance 추종
// AROM 전용 관절별 게인. motor_profiles 및 다른 모드의 게인은 변경하지 않는다.
//   0: AK80-64, 2: AK70-10. 미지정 관절(1, 3)은 기존 AROM 값을 유지한다.
static const float AROM_DOB_J[JOINT_COUNT]  = {0.010f, 0.0024f, 0.004f, 0.0f};   // [33차] j2 0.04 시도 → 실기 E-stop(09-14 AROM) → 0.004 원복. 임피던스 kp=kd=0 경로에서 AROM 은 135207 설정으로 동작 확인
static const float AROM_DOB_B[JOINT_COUNT]  = {0.050f, 0.0f,    0.011f, 0.0f};
// [39차-e] j1 0.005→0.040: 이득 1/M_v 가 200 이라 τ̂_h 0.28 Nm 만 들어와도 ddq_ref 가 즉시
//   상한(15 rad/s²)에 걸려 기준 궤적이 폭주했다. 1/25 로 낮춘다 — 반응이 둔해지는 대신 안정.
//   (방침: 회전관절은 부드러움보다 "동작" 우선. 너무 뻑뻑하면 이 값만 내리면 된다.)
static const float AROM_M_V_J[JOINT_COUNT]  = {0.010f, 0.040f, 0.045f, 0.005f};
static const float AROM_B_V_J[JOINT_COUNT]  = {0.100f, 0.40f,  0.450f, 0.15f};   // [39차-e] j1 0.15→0.40 (M_v 와 함께 상향)
static const float AROM_KP_IMP_J[JOINT_COUNT] = {55.0f, 70.0f, 22.0f, 70.0f};
static const float AROM_KD_IMP_J[JOINT_COUNT] = { 2.0f,  1.5f,  1.6f,  1.5f};
static const float AROM_DDQ_MAX = 15.0f;   // [9차] 가상 궤적 "가속도" 상한 [rad/s²] — τ̂_h 스파이크가
                                            //   M_v(0.005)로 나눠져 수천 rad/s² 로 튀는 것이 "점프"의 뿌리.
                                            //   15 = 사람 팔의 자연 가속 수준 (1 rad/s 도달 67 ms).
static const float AROM_DQ_MAX = 2.5f;     // 가상 궤적 속도 상한 [rad/s] — 0.5→2.5 (5차):
                                            //   AROM 은 능동 운동 — 손 push 는 1~3 rad/s 가 정상.
                                            //   0.5 로는 q_ref 가 못 따라와 스프링 벽+복원토크 발생.
static const float AROM_VEL_CUTOFF = 3.0f;  // AROM 전용 컷오프 [rad/s] — PROM 값(1.0, 수동 기준)
// [42차-b] 관절별 컷오프 — 0 이면 위 공통값을 쓴다(j0·j2·j3 거동 불변).
//   j1 만 5.0 (관절 72 °/s): 10-08 실기(121227)에서 |v|>3.0 이 **0.102 s** 지속되어 t=17.32 에
//   차단이 걸렸고, 이후 13.6 s 동안 기준이 −3.75° 에 얼어붙은 채 FIX 를 밀고 있었다. 원인은
//   AROM_TRIG_VMAX 를 차단값과 같은 3.0 으로 둔 것 — 기준이 차단선 속도까지 달리도록 허용했다.
//   ★트리거 방식에는 폭주 경로가 없다. 기준은 팔 뒤에 있으므로 빠른 움직임에서 PD 는 오히려
//   제동을 건다. 즉 고속은 고장이 아니라 사람이 빨리 민 것이고, 컷오프는 기구 고장 감지용으로만
//   남긴다. 토크 클램프(출력축 3.0)와 guard 벽은 그대로 작동한다.
//   (헬퍼 함수를 두면 Arduino 전처리기가 프로토타입을 MotorState 선언보다 앞에 끼워 넣어
//    컴파일이 깨지므로, 호출부에서 삼항식으로 직접 쓴다.)
// [42차-e] 5.0 → 7.0 (관절 100 °/s). 10-08 실기에서 **가벼워질수록 빨리 움직이게 되어**
//   최대 속도가 3.42 → 3.54 → 3.66 → 5.30 rad/s 로 올라갔고, 5.0 차단이 124023 에서 또 발동
//   (초과 0.102 s). 관측값을 쫓아 올리는 방식은 끝이 없으므로 **물리적 상한**으로 정한다:
//   신전 쪽 guard 여유가 1.5° 로 가장 빡빡하고, 벽 토크 상한 8 Nm(관절)로 그 안에 세우려면
//   운동에너지 ≤ 0.21 J → 관절 관성 0.1 kg·m²(추정) 기준 관절 117 °/s 가 한계.
//   7.0 (관절 100 °/s)은 실측 76 대비 여유 32%, 한계 117 아래로 마진 확보.
//   ★더 빨리 쓰게 되면 차단값이 아니라 +75° guard 여유(1.5°)가 진짜 제약이다 — guard 확대나
//   하드스톱 접촉 감수 여부를 판단해야 한다.
// [43차] j2 3.0(관절 172 °/s) → 3.5(200 °/s). 10-12 팔꿈치 AROM(134214)에서 **179.5 °/s 로
//   0.102 s 초과 → t=6.39 에 발동**, 이후 8.42 s 기준각 정지. 본 런(134317)은 120.4 °/s 로 무사.
//   상한 근거: 굴곡 쪽 guard −120° 에서 기구 한계 −124° 까지 4° = 0.0698 rad 이고, 위에서 켠
//   AROM_WALL(8.3 Nm) 로 그 안에 세우려면 운동에너지 ≤ 0.58 J → 관절 관성 0.1 kg·m²(추정)
//   기준 195 °/s 가 한계. 3.5(200)는 그 경계이므로 **벽을 켠 것이 전제**다.
//   j3 는 3.0 유지 — direct-drive 라 물리 스토퍼가 없어 guard 가 유일한 제한이고, 10-12 실측
//   최대도 159.8 °/s 로 여유가 있다.
static const float AROM_VEL_CUTOFF_J[JOINT_COUNT] = { 0.0f, 7.0f, 3.5f, 0.0f };
                                            //   재사용이 push 중 조용한 fix 전환(=hard+reverse) 유발.
// [jehyeon v2] τ̂_human 마찰 복원항의 저속 게이트 — 정지 시 속도 노이즈가 Fs 게인으로
//   증폭되어 자가구동(드리프트)하는 것 방지 (2차 시험에서 CW 자가이동 확인).
// [jehyeon v2, 6차] 이 구조는 disturbance 를 +로 인가하므로 마찰항 = 능동 마찰상쇄.
//   실측 Stribeck 100% 를 넣으면 순마찰≈0~음수 → 자가가속·컷오프 (실기 확인, 21:38).
//   → 운동마찰(Fc·tanh+Fv·v)만, 70% 스케일 (0.7×0.25 < 실측 0.25 → 항상 소산적).
static const float AROM_FRIC_SCALE  = 0.7f;
// [39차-b] ★AROM 전용 중력 — 관절 프레임 M·L [kg·m] 과 위상 [rad]. 0 이면 프로필 gc 를 쓴다.
//   j1 은 gc_m=0 이라 중력이 τ_ref 에 안 들어갔고, AROM 의 disturbance = J·a + 마찰(v) − LPF(τ_ref)
//   구조에서 **중력이 만든 움직임이 그대로 "사람 토크" 로 해석**되어 양의 되먹임이 된다.
//   실기: 손 대기 전에 스스로 움직임(10-07). 시작자세 q=0 에서 중력이 최대(위상 +99.8°)라
//   자세 전환 잔여 속도로 게이트가 열리면 중력 1.06 + 마찰보상 0.83 > 마찰 1.19 (관절 Nm) → 가속.
//   값: 09-07 CPM 1~7단 전진·후진 평균 LS 피팅 τ_g = 1.0798·sin(q +99.8°) +0.04 Nm (관절, rms 0.024).
//   ★CPM/PROM/isometric/isotonic 은 DOB 가 중력을 흡수하도록 검증돼 있으므로 프로필 gc 는 0 유지
//   (이중보상 방지) — 이 배열은 run_arom 에서만 쓴다.
static const float AROM_GC_ML   [JOINT_COUNT] = { 0.0f, 0.1101f, 0.0f, 0.0f };   // [kg·m] 관절
// [39차-c] AROM guard 벽과 τ̂_h 상한 — **0 이면 비활성**이라 j0·j2·j3 거동은 비트 단위로 종전과 같다.
//   10-07 실기: j1 AROM 에서 팔이 guard 를 14.9° 넘어 하드스톱(−32°)까지 끌려갔다. 임피던스 복원
//   토크(출력축 72.7)가 τ_ref 에 들어가 LPF(τ_ref) 로 되돌아오며 disturbance 가 그대로 상쇄 →
//   τ_cmd ≈ 0, 벽 무력화, τ̂_h 는 관절 −292 Nm 로 와인드업. **DOB 구조상 τ_ref 에 넣은 항은 DC 에서
//   전부 상쇄되므로, 벽과 중력은 DOB 밖에서 더해야 한다.** 계수는 isotonic 소프트 월 검증값과 동일.
// [43차] j2·j3 도 활성화. 근거: 10-12 손목 AROM(135917)에서 **guard 를 3.38° 침투**(밖에 1.84 s).
//   기존 어드미턴스 AROM 의 되돌림은 임피던스 PD 인데 그것이 τ_ref 에 들어가 DC 에서 상쇄되므로
//   (#39 구조 성질 ①) 사실상 벽이 약하다. 이 배열은 **DOB 밖에서** 가산되어 상쇄되지 않는다.
//   값은 각 관절의 ISOT_WALL 검증값과 동일(elbow 40/2.0/8.3, 손목 30/1.0/3.0).
//   j0 은 아직 0 — 10-12 런에서 guard 를 1.25° 넘었으나 기구 스토퍼(−185°)까지 5° 여유가 있어
//   보류. 필요해지면 ISOT_WALL[0] 값(60/4.0/15)을 그대로 쓰면 된다.
static const float AROM_WALL_KP     [JOINT_COUNT] = { 0.0f, 7.5f,  40.0f, 30.0f };   // [Nm/rad] 출력축
static const float AROM_WALL_KD     [JOINT_COUNT] = { 0.0f, 0.25f, 2.0f,  1.0f };
static const float AROM_WALL_TAU_MAX[JOINT_COUNT] = { 0.0f, 2.0f,  8.3f,  3.0f };   // j1 은 출력축(관절 8 Nm)
static const float AROM_TAU_H_MAX   [JOINT_COUNT] = { 0.0f, 3.0f,  0.0f, 0.0f };   // τ̂_h 상한(출력축)

// ============================================================================
// [40차] ★사용자 트리거 PROM 방식 AROM (관절별 선택). 1 이면 run_arom_trigger() 를 쓴다.
//   배경: 0522 어드미턴스+임피던스 구조는 "작동점에서 중력 ≈ 0" 을 암묵 전제한다. j0(팔 수하)·
//   j2(신전)는 시작자세 중력이 0 이라 성립하지만, j1 은 측정 위상이 +99.8° 라 **시작자세가 중력
//   최대점**(관절 1.06 Nm)이다. 마찰 1.19 중 70%를 AROM 이 상쇄하면 남는 저항이 0.36 뿐이라
//   손 대기 전에 팔이 움직이고, 그 순간 τ̂_h 의 −LPF(τ_ref) 항(임피던스 반작용 복원 채널)이
//   양의 되먹임으로 돌아 폭주한다. 10-07 네 차례 재현(−29.9° 하드스톱 / +36° / +42° / +61.7°).
//   중력을 τ_ref 안·밖·관측기 세 자리에 넣어 봤으나 각각 상쇄·오인·계단이탈로 모두 실패.
//   → j1 은 **추정을 포기하고 판정만** 한다. PROM 의 PD + 기준램프 구조를 쓰되, 램프를 시간이
//     아니라 **위치 오차**로 전진시킨다. 기준은 팔을 따라가기만 하므로(절대 앞서지 않음)
//     구조적으로 음의 되먹임이고, 손을 놓으면 PD 가 진행 반대로 당겨 제동한다.
static const bool  AROM_TRIG_J  [JOINT_COUNT] = { false, true, false, false };
//   불감대 [rad, 출력축]: 0.05 → PD 몫 5.0*0.05 = 0.25(출력축) = 관절 1.0 Nm.
//   마찰 1.19 와 합쳐 관절 2.2 Nm = 레버 0.294 m 에서 손 약 0.8 kgf 가 트리거 힘이다.
// [40차-b] 1차 실기(10-07 072456): 체감이 무거웠고 원인은 불감대가 아니라 **기준 속도 상한**
//   이었다. 사용자가 관절 6.5 °/s(최대 15)로 미는데 상한이 5.7 °/s 라 기준이 뒤처져 위치오차가
//   평균 1.75°(불감대 0.72° 의 2.4배)까지 벌어졌고, PD 몫이 설계 1.0 → 실측 2.44 Nm(최대 3.90).
//   → VMAX 0.4→1.2 (관절 17 °/s), GAIN 8→20 (상한 도달까지 초과오차 0.86° 면 충분),
//     DEAD 0.05→0.03 (관절 0.43°, PD 몫 0.6 Nm).
// [40차-c] 2차 실기(073139)로 정상상태 오차식을 검증: **err = DEAD + v_user/GAIN**.
//   측정 0.43° + 13.4°/s÷20 = 1.10° vs 실측 1.115° (일치). 남은 저항의 주범은 PD 의 속도항
//   KP·v/GAIN 이었다(관절 0.94 Nm). 또 최대 24.9 °/s 로 밀어 상한 17 을 넘긴 구간이 있었다.
//   → GAIN 20→80 (속도항 1/4), DEAD 0.03→0.015 (불감대 몫 0.6→0.3), VMAX 1.2→2.0 (관절 29 °/s).
//   GAIN·dt = 0.16 ≪ 2 라 이산 안정성 여유는 충분하고, 기준은 여전히 팔을 따라가기만 한다.
// [42차] 투명도 향상 — 10-08 실기(120133 비착용 / 120443 착용)에서 체감 0.64 / 0.59 kgf.
//   어제(0.44)보다 높은 이유는 더 빠르게 밀어(평균 23.7 / 26.6, 최대 41.8 °/s) **VMAX 를 넘겨**
//   오차가 쌓인 것(0.512° 예상 vs 실측 1.055°). 저항 = 마찰 − 마찰FF + KP·오차,
//   오차 = DEAD + v/GAIN 이므로 세 파라미터를 함께 내린다.
//   ★이 셋은 안전과 무관하다 — 기준은 팔을 따라가기만 하므로 자가구동 경로가 없다. 자가구동
//   여부를 결정하는 것은 AROM_TRIG_FRIC 하나뿐이고 그것은 건드리지 않는다(여유 30% 유지).
//   VMAX 3.0 = 과속 차단(AROM_VEL_CUTOFF)과 같은 값이라 더 올릴 이유가 없다.
//   GAIN·dt = 0.30 ≪ 2 로 이산 안정 여유 충분. DEAD 0.008 rad = 관절 0.115° (엔코더 노이즈 상회).
//   예상 체감: 26.6 °/s 기준 오차 0.957° → 0.292°, 0.59 → 0.27 kgf.
static const float AROM_TRIG_DEAD [JOINT_COUNT] = { 0.0f, 0.008f, 0.0f, 0.0f };
// [42차-c] GAIN 150→300 — 반전 응답 + 투명도. 기준 뒤처짐 = DEAD + v/GAIN 이므로 GAIN 이
//   두 가지를 동시에 결정한다: ① 미는 힘(KP·오차) ② 방향 전환 시 유격(기준을 지나 반대쪽
//   불감대까지 가야 기준이 따라오고, 그 전까지 PD 가 반대로 당긴다).
//   36 °/s 기준 뒤처짐 0.355°→0.235°, 반전 유격 0.47°→0.35°, 미는 힘 0.39→0.24 kgf.
//   ★자가구동 경로는 생기지 않는다 — 기준은 여전히 팔을 따라가기만 하고 FRIC 은 불변.
//   이산 안정: 오차가 매 틱 (1 − GAIN·dt) = 0.4 배로 감쇠 (발산 경계 2.0).
//   주의: 위치 노이즈가 GAIN 배로 기준속도에 실려 KD 항을 통해 버징으로 느껴질 수 있다.
//   느껴지면 200 으로 내린다.
static const float AROM_TRIG_GAIN [JOINT_COUNT] = { 0.0f, 300.0f, 0.0f, 0.0f };   // 초과오차→기준속도 [1/s]
// [42차-d] VMAX 3.0→5.0 = **AROM_VEL_CUTOFF_J[1] 과 같은 값**. 앞서 "VMAX 를 차단값과 같게 둬서
//   차단이 걸렸다"고 적었으나 **인과가 틀렸다** — 차단은 팔의 속도로 판정하므로 VMAX 와 무관하다
//   (121227 은 당시 차단값 3.0 을 팔이 3.42 로 넘긴 것). VMAX 는 안전 장치가 아니다: 기준은 팔을
//   따라가기만 하고 v_ref 가 팔보다 빠르면 간격이 줄어 v_ref 도 줄므로 스스로 제한된다.
//   유일한 역할은 guard 복귀 등으로 오차가 클 때 dq_ref 가 KD 항(0.15·(dq_ref − v))으로 폭증하는
//   것을 묶는 것. 최악값 0.15×5.0 = 0.75(출력축) = 관절 3 Nm 로 토크 클램프 3.0(출력축) 안.
//   → 논리적 상한은 차단값이다. 그보다 빠르면 모드가 멈추므로 추종할 이유가 없다.
//   실측 최대 3.66 rad/s(관절 52 °/s) 대비 여유 37%. 착용 체감 0.43 → 0.24 kgf 예상.
static const float AROM_TRIG_VMAX [JOINT_COUNT] = { 0.0f, 7.0f,   0.0f, 0.0f };   // [42차-e] 차단값과 동일. 관절 100 °/s
// [40차-b] 운동마찰 피드포워드 — 측정 최솟값(ST_FC)의 50% 만. 관측기가 없으므로 오인될 주체가
//   없고, 실제 마찰(≥ST_FC)보다 항상 작아 자가구동이 구조적으로 불가능하다. 저속 게이트는
//   기존 AROM 과 동일(AROM_FRIC_V_ON/FULL) — 정지 중에는 0 이라 홀드를 흔들지 않는다.
static const float AROM_TRIG_FRIC [JOINT_COUNT] = { 0.0f, 0.7f,  0.0f, 0.0f };   // [40차-c] 0.5→0.7 (기존 AROM_FRIC_SCALE 과 동일 여유)
static const float AROM_GC_PHASE[JOINT_COUNT] = { 0.0f, 1.7418f,  0.0f, 0.0f };   // [rad] = +99.8 deg
static const float AROM_FRIC_V_ON   = 0.05f;   // 이 미만: 마찰 복원 0 — 위치차분 속도 노이즈(±0.02~0.05) 위로 (7차: 정지 채터 제거)
static const float AROM_FRIC_V_FULL = 0.12f;   // 이 이상: 마찰 복원 100%
static const uint32_t CPM_END_HOLD_MS = 10000;
// [16차] 관절별 게인으로 변경 — 기존 상수 Kd=2 는 elbow(J=0.0103) 기준: 어깨(J=0.452,
//   44배)에선 감쇠비 ζ≈0.12 로 3 Hz 발진 (실기 확인). 어깨 Kd=12 → ζ≈0.73.
// [24차] j1 = {50, 0.5}: sector 기어가 non-backdrivable 이라 홀드 자체는 기어가 담당 —
//   고강성 불필요 + 피니언-sector backlash 에 Kp=150 은 버징(0625 경고) 위험. 출력축 K=50
//   은 관절강성 ×16 = 800 Nm/rad 상당, 클램프 1.0 Nm 에서 0.02 rad 부터 포화. 실기 튜닝 대상.
static const float ISOMETRIC_KP_J[4] = {200.0f,  50.0f, 200.0f, 150.0f};
static const float ISOMETRIC_KD_J[4] = { 14.0f,   0.5f,   2.5f,   2.0f};
static const uint8_t ISOMETRIC_TOTAL_REPS = 3;
static const uint32_t ISOMETRIC_REST_MS = 3000;
static const float gc_G = 9.80665f;

static const float fc_vel = 40.0f;
static const float tau_vel = 1.0f / (2.0f * M_PI * fc_vel);
static const float alpha_vel = DOB_Ts / (tau_vel + DOB_Ts);

static const float fc_Q = 4.0f;
static const float tau_Q = 1.0f / (2.0f * M_PI * fc_Q);
static const float alpha_Q = DOB_Ts / (tau_Q + DOB_Ts);

// ============================================================================
// [Motor Communication & State]
// ============================================================================
MD80Comm motor;
bool motor_enabled = false;

enum class CPMState { BUILD, RUN, HOLD };
enum class AROMState { IDLE, RUNNING };
enum class IsometricState : uint8_t {
    IDLE = 0,
    MOVING_TO_TARGET = 1,
    HOLDING = 2,
    RESTING = 3,
    COMPLETED = 4
};
enum class IsotonicState { MOVING_TO_TARGET, ACTIVE };

// 단일 변수로 존재하던 제어 상태를 배열(Struct)로 묶어 독립화
struct MotorState {
    uint8_t prev_mode = CMD_IDLE;
    uint32_t last_command_id = 0;

    float current_position = 0.0f;
    float current_velocity = 0.0f;
    float prev_raw_position = 0.0f; // 소프트웨어 속도 계산용
    float motor_output_torque = 0.0f;
    float real_torque = 0.0f;
    bool online = false;

    // DOB Variables
    float filtered_velocity = 0.0f;
    float prev_filtered_velocity = 0.0f;
    float acceleration = 0.0f;
    float tau_nominal_plant = 0.0f;
    float filtered_tau_nominal_plant = 0.0f;
    float estimated_disturbance = 0.0f;
    float target_torque = 0.0f;
    float prev_target_torque = 0.0f;
    float filtered_prev_target_torque = 0.0f;

    // Mode Flags
    bool g_cpmMode = false;
    bool g_promMode = false;
    bool g_aromMode = false;
    bool g_isometricMode = false;
    bool g_isotonicMode = false;
    bool g_fixMode = false;

    // Isotonic Variables
    float isotonic_resistance = 0.0f; // 가상 덤벨 질량 [kg]
    float isotonic_soft_gain = 0.0f;
    bool  isot_at_wall = false;      // [36차] 소프트 월 진입 알림 1회용 (히스테리시스 0.03 rad)

    // PROM Variables
    float isotonic_target_angle = 0.0f;
    IsotonicState isotonic_state = IsotonicState::MOVING_TO_TARGET;
    float isotonic_move_ref = 0.0f;     // [jehyeon v2] MOVING 단계 램프 참조

    bool prom_inited = false;
    float prom_max_position = 0.0f;
    float prom_min_position = 0.0f;
    float prom_velocity_ref = -PROM_DEFAULT_SPEED_RAD_S;
    float prom_velocity_integral = 0.0f;
    uint32_t prom_stall_start_ms = 0;   // [jehyeon] stall 감지 타이머
    bool prom_at_limit = false;         // [jehyeon] guard 도달 1회 알림용
    // [jehyeon v2] 0716형 PROM 상태 (기준위치 램프 + Stribeck DOB)
    float arom_tau_human = 0.0f;        // [jehyeon v2] 직전 루프 disturbance = τ̂_human (0522 Step 2)
    float prom_ref_pos = 0.0f;          // 기준위치 적분기 [rad, 절대각]
    float prom_vel_ramp = 0.0f;         // 램프된 기준속도 [rad/s]
    float nl_filt_vel = 0.0f,    nl_past_filt_vel = 0.0f;
    float nl_filt_torque = 0.0f, nl_past_filt_torque = 0.0f;
    float nl_est_torque = 0.0f,  nl_past_est_torque = 0.0f;
    float nl_tau_cmd = 0.0f;
    float nl_last_dist = 0.0f;   // [21차] 마지막 Stribeck-DOB disturbance (SD 로깅용)
    // [33차] 임피던스 모드 참조 (IMP_JOINT 관절만 사용)
    float imp_q_ref = 0.0f, imp_v_ref = 0.0f;
    bool  imp_use_pd = true;        // true: 드라이버 PD 사용(참조 모드) / false: kp=kd=0, τ_ff 로 토크 직결
    bool  imp_pd_gains_on = true;   // 드라이버에 현재 써진 kp/kd 상태

    // CPM Variables
    float P_MIN = 0.0f, P_MAX = 0.0f;
    float last_end = 0.0f;
    CPMState cpm_state = CPMState::BUILD;
    int cpm_dir = -1;
    uint32_t cpm_hold_ts = 0;
    uint32_t cpm_run_ts = 0;
    bool cpm_inited = false;
    float cpm_pos_ref = 0.0f;
    float cpm_vel_ref = 0.0f;
    float cpm_acc_ref = 0.0f;
    float cpm_start_pos = 0.0f;
    float cpm_t_acc_s = 1.0f;
    float cpm_t_dcc_s = 1.0f;
    float cpm_t_total_s = 2.0f;
    float cpm_profile_vel = 0.0f;
    
    float CPM_V_MAX = 0.3f;
    float CPM_T_ACC = 1000.0f; // ms

    // AROM Variables
    AROMState arom_state = AROMState::IDLE;
    float arom_max_position = 0.0f;
    float arom_min_position = 0.0f;
    float arom_init_position = 0.0f;
    float q_ref_adm = 0.0f;
    float dq_ref_adm = 0.0f;

    // Isometric Variables
    IsometricState isom_state = IsometricState::IDLE;
    float isom_target_angle = 0.0f;
    float isom_move_ref = 0.0f;
    float isom_move_speed = POSTURE_SPEED_RAD_S;
    uint32_t isom_hold_duration_ms = 0;
    uint32_t isom_phase_start_ms = 0;
    uint8_t isom_rep = 0;

    // FIX_POS Variables
    float fix_target_pos = 0.0f;
    float current_target_pos = 0.0f;
};

MotorState m_states[JOINT_COUNT];
bool expected_online[JOINT_COUNT] = {false, false, false, false};

uint32_t getIsometricRemainingMs(const MotorState& s) {
    uint32_t duration_ms = 0;
    if (s.isom_state == IsometricState::HOLDING) {
        duration_ms = s.isom_hold_duration_ms;
    } else if (s.isom_state == IsometricState::RESTING) {
        duration_ms = ISOMETRIC_REST_MS;
    } else {
        return 0;
    }

    const uint32_t elapsed_ms = millis() - s.isom_phase_start_ms;
    return (elapsed_ms >= duration_ms) ? 0 : (duration_ms - elapsed_ms);
}

// [추가] 자세 변환이 완료되었는지 확인하는 전역 플래그
bool posture_ready = true;
int posture_active_joint = -1;

float getStartPostureRad(int selected_joint, int joint) {
    static const float START_POSTURE_DEG_TO_RAD = M_PI / 180.0f;
    // [24차] rShoulderRo 행 {90,0,90,0} → {0,0,-90,0} 정정: 기존 값은 두 관절 모두 guard 밖
    //   (당시 guard EF -115..+5°, elbow -95..+5°) + 이 기구 프레임에서 부호 반대 — posture 가 하드스톱을
    //   프로필 전체 토크로 밀어붙일 위험. 내외회전 표준자세 = 상완 연직(EF 0 — 회전축 연직
    //   유지, j1 무중력 전제) + 팔꿈치 90° 굴곡(이 기구는 굴곡이 음수 = -90°). 값은 관절
    //   프레임 deg — j1 적용 시 JOINT_GEAR 환산은 사용처에서. rWrist 행도 동일 부호 오류 정정.
    static const float start_posture_deg[JOINT_COUNT][JOINT_COUNT] = {
        {0.0f, 0.0f, 0.0f, 0.0f},    // rShoulderEF
        // [31차] rShoulderRo: EF 0 → -90 (사용자 요청 09-07). 상완 수평(전방 거상 90°) + 팔꿈치 90° 굴곡에서
        //   내외회전. 주의: 회전축이 수평이 되어 j1 에 중력토크 최대 ~1.0 Nm(관절, MJCF q0=-90/q2=-90) 발생 —
        //   j1 gc_m=0 이라 DOB 가 흡수(출력축 0.25 Nm, 클램프 3 안). j0 는 -90° 홀드(FIX 48 Nm)로 유지.
        {-90.0f, 0.0f, -90.0f, 0.0f},  // rShoulderRo
        {0.0f, 0.0f, 0.0f, 0.0f},    // rElbow
        {0.0f, 0.0f, -90.0f, 0.0f}   // rWrist: 팔꿈치 90° 굴곡(전완 수평)에서 손목 굴곡/신전 [35차 활성]
    };
    if (selected_joint < 0 || selected_joint >= JOINT_COUNT ||
        joint < 0 || joint >= JOINT_COUNT) return 0.0f;
    return start_posture_deg[selected_joint][joint] * START_POSTURE_DEG_TO_RAD;
}

// ============================================================================
// [Ticker for Loop]
// ============================================================================
mbed::Ticker t_control;
volatile bool tick_control = false;
void onControlTick() { tick_control = true; }

// ============================================================================
// [Helper Functions]
// ============================================================================
void resetDOB(int i) {
    MotorState& s = m_states[i];
    s.filtered_velocity = 0.0f;
    s.prev_filtered_velocity = 0.0f;
    s.acceleration = 0.0f;
    s.tau_nominal_plant = 0.0f;
    s.filtered_tau_nominal_plant = 0.0f;
    s.estimated_disturbance = 0.0f;
    s.prev_target_torque = 0.0f;
    s.filtered_prev_target_torque = 0.0f;
    s.target_torque = 0.0f;
    s.cpm_inited = false;

    // [jehyeon v2] 0716형 PROM 상태 리셋
    s.arom_tau_human = 0.0f;
    s.prom_ref_pos = 0.0f;
    s.prom_vel_ramp = 0.0f;
    s.nl_filt_vel = 0.0f;    s.nl_past_filt_vel = 0.0f;
    s.nl_filt_torque = 0.0f; s.nl_past_filt_torque = 0.0f;
    s.nl_est_torque = 0.0f;  s.nl_past_est_torque = 0.0f;
    s.nl_tau_cmd = 0.0f;
    s.nl_last_dist = 0.0f;
    s.prom_stall_start_ms = 0;
    s.prom_at_limit = false;
}

float clamp_torque(int i, float tau) {
    float max_t = motor_profiles[i].max_torque;
    if (tau > max_t) return max_t;
    if (tau < -max_t) return -max_t;
    return tau;
}

// [jehyeon 2026-07-18 v2] 관절별 Stribeck 마찰 모델 (0611/0716 동일 식, smoothSign eps=0.08)
// [30차] 참조속도 기반 마찰 FF (상수는 FF_REF_* 참조)
static inline float fricFFRef(int i, float v_ref) {
    if (FF_REF_SCALE[i] <= 0.0f) return 0.0f;
    return FF_REF_SCALE[i] * (FF_REF_FC[i] * tanhf(v_ref / 0.08f) + FF_REF_FV[i] * v_ref);
}
// [33차] 중력 FF (FIX 전용값 우선, 없으면 프로필) — 임피던스 τ_ff 용
static inline float gravityFF(int i, float q) {
    const MotorProfile& p = motor_profiles[i];
    const float ml = (FIX_GC_ML[i] > 0.0f) ? FIX_GC_ML[i] : (p.gc_m * p.gc_l);
    return (ml != 0.0f) ? (ml * gc_G * sinf(q + GC_PHASE_RAD[i])) : 0.0f;
}
// [30차] 디더 — 관절별 위상 누적(호출 tick 당 1회 진행). 이동 중에만.
static inline float ditherTorque(int i, float v_ref) {
    static float phase[JOINT_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (DITHER_A[i] <= 0.0f || fabsf(v_ref) < 1e-4f) { phase[i] = 0.0f; return 0.0f; }
    phase[i] += 2.0f * (float)M_PI * DITHER_HZ * DOB_Ts;
    if (phase[i] > 2.0f * (float)M_PI) phase[i] -= 2.0f * (float)M_PI;
    return DITHER_A[i] * sinf(phase[i]);
}

static inline float stribeckFriction(int i, float v) {
    if (ST_FC[i] <= 0.0f && ST_FS[i] <= 0.0f) return ST_FV[i] * v;   // 미식별 → 점성만
    float ratio = fabsf(v) / ST_VS[i];
    float st = ST_FC[i] + (ST_FS[i] - ST_FC[i]) * expf(-(ratio * ratio));
    return st * tanhf(v / 0.08f) + ST_FV[i] * v;
}

bool isSupportedActiveJoint(int i) {
    // [37차] `i < CONNECTED_MOTOR_COUNT` 제거 — 스캔 결과는 연속이 아닐 수 있다
    //   (예: 110+120 만 연결 = j0,j2). 연결 여부만 본다.
    return i >= 0 && i < JOINT_COUNT && MOTOR_CONNECTED[i];
}

bool isMotorConnected(int i) {
    return i >= 0 && i < JOINT_COUNT && MOTOR_CONNECTED[i];
}

bool isActiveMode(uint8_t mode) {
    return mode >= CMD_AROM && mode <= CMD_ISOT;
}

// [38차] ISOT 거부 해제. #24 의 거부 사유 "회전축 연직 → 중력토크 0 → sin(q) 가상 덤벨 무의미"
//   는 #31(09-07)에서 Ro 시작자세를 {-90,0,-90,0} 으로 바꿔 **상완이 수평 = 회전축도 수평**이
//   되면서 무효가 되었다 (j1 에 관절 ~1.0 Nm 의 실제 중력 사인파가 걸린다 — start_posture_deg
//   주석 참조). 프레임 환산과 레버 암을 바로잡아 활성화한다.
// [39차] AROM 거부도 해제. 거부 사유였던 "ST_* 는 bare 값이라 τ̂_human 추정 불성립" 을 09-07
//   CPM 10단 로그로 조립 상태 재식별하여 해소했다(ST_FC 0.105→0.297, 주석 참조).
//   보상은 ROM 최솟값 기준이라 어느 각도에서도 과보상이 아니다 — 뻑뻑한 끝(+75°)에서는 관절
//   0.62 Nm 가 사용자에게 남는다(무보상 1.45 대비 57% 경감). 투명하지는 않지만 밀면 움직인다.
bool isModeSupportedForJoint(int i, uint8_t mode) {
    return true;
}

// [37차] 버스 스캔 — 각 규약 ID 를 ping 하여 실제 연결된 관절만 운전 대상으로 삼는다.
//   ★판정은 보수적으로: 응답이 한 번이라도 오면 present, 8회 모두 실패해야 absent.
//   실재하는 모터를 absent 로 오판하면 그 관절은 enable 되지 않아 힘이 빠진 채 남는다
//   (종전의 "크게 틀리면 E-stop" 과 달리 조용한 실패라 더 위험 — 그래서 재시도를 넉넉히).
static const int      SCAN_ATTEMPTS = 8;
static const uint32_t SCAN_RETRY_MS = 3;

void scanMotorBus() {
    CONNECTED_MOTOR_COUNT = 0;
    for (int i = 0; i < JOINT_COUNT; i++) {
        motor.setMotorID(motor_profiles[i].id);
        motor.flushRxFifo();
        bool present = false;
        float pos = 0.0f;
        for (int a = 0; a < SCAN_ATTEMPTS && !present; a++) {
            if (READ_GAP_US[i]) delayMicroseconds(READ_GAP_US[i]);
            present = motor.getPosition(pos);
            if (!present) delay(SCAN_RETRY_MS);
        }
        MOTOR_CONNECTED[i] = present;
        if (present) CONNECTED_MOTOR_COUNT++;
        Serial.print("[SCAN] joint "); Serial.print(i);
        Serial.print(" ID "); Serial.print(motor_profiles[i].id);
        Serial.print(" ("); Serial.print(motor_profiles[i].name); Serial.print(") ");
        if (present) { Serial.print("PRESENT Position="); Serial.println(pos, 4); }
        else         { Serial.println("absent"); }
    }
    Serial.print("[SCAN] connected "); Serial.print(CONNECTED_MOTOR_COUNT);
    Serial.println("/4");
}

// [37차] 기구 모듈 구성: **어깨 모듈(j0+j1) / 엘보우 모듈(j2) / 손목 모듈(j3)**.
//   모듈은 몸쪽부터 순서대로만 붙으므로 유효 구성은 세 가지뿐이다:
//     어깨 / 어깨+엘보우 / 어깨+엘보우+손목
//   그 밖의 스캔 결과는 모듈 제거가 아니라 **배선 이상**이다. 특히 어깨 두 축 중 하나만
//   응답하는 경우는 한 축이 enable 되지 않아 힘이 빠진 채 팔을 드는 상태가 되므로 위험하다.
//   → enable 을 거부하고 1 s 뒤 재시도(케이블을 다시 꽂으면 스스로 복구된다).
//   반환: 유효하면 모듈 수(1~3), 무효하면 −1.
int checkModuleConfig() {
    const bool sh0 = MOTOR_CONNECTED[0], sh1 = MOTOR_CONNECTED[1];
    const bool el  = MOTOR_CONNECTED[2], wr  = MOTOR_CONNECTED[3];
    if (sh0 != sh1) {
        Serial.print("[ERROR] Shoulder module is one unit (j0+j1) but only ");
        Serial.print(sh0 ? "ID 110" : "ID 130");
        Serial.println(" answered -- check the other cable. Not enabling.");
        return -1;
    }
    if (!sh0) {
        Serial.println("[ERROR] Shoulder module (ID 110+130) absent -- it is the base module. Not enabling.");
        return -1;
    }
    if (wr && !el) {
        Serial.println("[ERROR] Wrist present but elbow (ID 120) absent -- wrist mounts distal to elbow. Check cable. Not enabling.");
        return -1;
    }
    const int n = 1 + (el ? 1 : 0) + (wr ? 1 : 0);
    Serial.print("[CONFIG] module set = shoulder");
    if (el) Serial.print(" + elbow");
    if (wr) Serial.print(" + wrist");
    Serial.print("  ("); Serial.print(n); Serial.println(" module(s))");
    return n;
}

// [37차] 모듈 구성에 따라 달라지는 파라미터 보정. 중력모델은 "그 관절" 이 아니라
//   "말단에 무엇이 달려 있는가" 에 의존한다 — #32 에서 손목 구조물(AK60-6 포함)을 달며
//   j2 의 gc_m·gc_l 이 0.0254 → 0.1643 kg·m (6.5배, τ_g 0.249 → 1.61 Nm @ −90°) 로 바뀌었다.
//   손목을 떼고 팔꿈치만 운용하면 0.1643 은 1.36 Nm 과보상(팔을 들어올리는 편향)이 된다.
//   j2 는 DOB off 라 τ_g 가 직접 작용하므로 반드시 전환해야 한다. j0 은 DOB 가 미모델
//   중력을 전담(#26)해 자가 흡수하고, j1·j3 은 gc=0 이라 해당 없음.
void applyModuleConfig() {
    // [37차] 엘보우 자신이 없으면 바꿀 이유가 없다 (2모터 구성에서 오해 소지 메시지 방지).
    if (MOTOR_CONNECTED[2] && !MOTOR_CONNECTED[3] && motor_profiles[2].gc_l != 0.0f) {
        motor_profiles[2].gc_m = ELBOW_GC_ML_NO_WRIST / motor_profiles[2].gc_l;
        FIX_GC_ML[2]    = ELBOW_GC_ML_NO_WRIST;
        GC_PHASE_RAD[2] = 0.0f;   // −11° 위상은 손목 모터 COM 오프셋에서 온 값
        Serial.print("[CONFIG] wrist MODULE assumed removed -> elbow gravity M*L 0.1643 -> ");
        Serial.print(ELBOW_GC_ML_NO_WRIST, 4);
        Serial.println(" kg*m, phase -11 deg -> 0");
        // ★전제: 손목 "모듈"(모터+링크)이 기구에서 분리된 상태. 모듈은 그대로 둔 채 케이블만
        //   뽑으면 질량은 남아 있으므로 이 전환이 1.36 Nm 부족보상이 된다 (엘보우가 처짐).
        Serial.println("[CONFIG]   NOTE: cable-only disconnect with module still mounted makes this WRONG (-1.36 Nm)");
    }
}

// [37차] 선택 부위의 시작자세가 0 이 아닌 관절(= 팔을 그 자세로 만들어야 모드 전제가
//   성립하는 관절)이 빠져 있으면 그 인덱스를, 모두 있으면 −1 을 돌려준다. 예를 들어
//   rShoulderRo 는 j0 −90°·j2 −90° 로 상완을 수평으로 만들어야 회전축이 수평이 되고,
//   그 전제 위에서 중력·guard 가 성립한다.
int missingPostureJoint(int selected_joint) {
    for (int j = 0; j < JOINT_COUNT; j++) {
        if (getStartPostureRad(selected_joint, j) != 0.0f && !isMotorConnected(j)) return j;
    }
    return -1;
}

void readMotorState(int i) {
    MotorState& s = m_states[i];
    // 읽어온 값이 있다면 online 처리, 실패 시 이전 값 유지
    if (motor.getPosition(s.current_position) && 
        motor.getVelocity(s.current_velocity) && 
        motor.getTorque(s.real_torque)) {
        s.online = true;
    } else {
        s.online = false;
    }
}

// ============================================================================
// [Control: CPM]
// 팀원 S-curve(cosine) 궤적의 BUILD→RUN→HOLD FSM 으로 P_MIN↔P_MAX 왕복 반복.
// 제어법칙은 0611 방식: 관절별 PD(PROM_PD_KP/KD) + Stribeck DOB(τ=PD−dist−τ_st+τ_g).
// 엔드포인트는 관절 guard 로 클램프, BUILD tick 도 제어 지속(토크 공백 없음, 13차).
// [24차] 0625 안전층 추가: 과속 컷오프 + 추종오차 stall → 홀드 (아래 주석 참조).
// ============================================================================
float run_fix_pos(int i);   // 전방 선언 (컷오프/stall 시 홀드 전환용 — 정의는 아래)

float run_cpm(int i) {
    // [jehyeon 2026-07-18 v2] 궤적(팀원의 온라인 S-curve — 0611 과 동일 수식)은 유지하고
    //   제어법칙만 0611_cpm_elbow 로 교체: 관절별 PD(PROM_PD_KP/KD 공용) + Stribeck DOB
    //   (ST_* 실측 테이블, Tf=1/200). 구식 J·B DOB(alpha_Q 4Hz) 는 CPM 경로에서 제거.
    //   왕복은 팀원 방식(무한, stop 으로 종료) 유지 — 10 s 홀드 후 방향 반전.
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    // 속도 LPF (0611: 1차, Tf = 1/200) — PROM 과 동일한 nl_* 상태 공유 (모드 진입 시 resetDOB)
    s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);

    // [24차] 과속 컷오프 (0625 안전 2 — PROM 과 동일 정책): 자동 왕복 모드가 sector 하드스톱
    //   달린 j1 에도 열리면서 이식. 전 관절 공통이지만 검증 운전 속도(≤0.3 rad/s)의 3배라
    //   기존 elbow/어깨 동작엔 불변.
    if (fabsf(s.nl_filt_vel) > PROM_VEL_CUTOFF_J[i]) {
        s.g_cpmMode = false;
        s.g_fixMode = true;
        s.fix_target_pos = s.current_position;
        s.current_target_pos = s.current_position;
        Serial.print("[CPM] VEL CUTOFF joint ");
        Serial.print(i);
        Serial.println(" -> hold (resend cpm to retry)");
        return run_fix_pos(i);
    }

    float tau_g = (p.gc_m != 0.0f) ? (p.gc_m * p.gc_l * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;

    if (s.cpm_state == CPMState::BUILD) {
        // 참고 스케치와 같은 사인형 가감속 S-curve를 현재 위치에서 온라인 생성합니다.
        s.cpm_start_pos = s.current_position;
        s.cpm_pos_ref = s.cpm_start_pos;
        s.cpm_vel_ref = 0.0f;
        s.cpm_acc_ref = 0.0f;
        const float end = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
        const float distance = end - s.cpm_start_pos;
        const float direction = (distance >= 0.0f) ? 1.0f : -1.0f;
        s.cpm_t_acc_s = fmaxf(s.CPM_T_ACC * 0.001f, DOB_Ts);
        float peak_speed = fmaxf(s.CPM_V_MAX, 0.001f);
        if (fabsf(distance) < peak_speed * s.cpm_t_acc_s) {
            peak_speed = fabsf(distance) / s.cpm_t_acc_s;
            s.cpm_t_dcc_s = s.cpm_t_acc_s;
        } else {
            s.cpm_t_dcc_s = fabsf(distance) / peak_speed;
        }
        s.cpm_profile_vel = direction * peak_speed;
        s.cpm_t_total_s = s.cpm_t_dcc_s + s.cpm_t_acc_s;
        s.cpm_run_ts = millis();
        s.cpm_state = CPMState::RUN;
        // [13차] 기존 `return 0.0f` 제거 — BUILD tick 의 2 ms 토크 공백이 어깨(중력 ~4 Nm,
        //   DOB 가 유지 중)에서 방향 전환 순간 스파이크로 나타남 (elbow 는 0.4 Nm 라 무감).
        //   fall through: pos_ref = 시작위치 홀드로 제어 지속 → 토크 연속.
    }
    else if (s.cpm_state == CPMState::RUN) {
        const float end = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
        const float t = (millis() - s.cpm_run_ts) * 0.001f;
        const float T_acc = s.cpm_t_acc_s;
        const float T_dcc = s.cpm_t_dcc_s;
        const float V = s.cpm_profile_vel;
        const float a_peak = (M_PI * V) / (2.0f * T_acc);

        if (t <= T_acc) {
            const float phase = t / T_acc;
            s.cpm_acc_ref = a_peak * sinf(M_PI * phase);
            s.cpm_vel_ref = V * (1.0f - cosf(M_PI * phase)) * 0.5f;
            s.cpm_pos_ref = s.cpm_start_pos
                + V * (t * 0.5f - T_acc * sinf(M_PI * phase) / (2.0f * M_PI));
        } else if (t <= T_dcc) {
            const float d_acc = V * T_acc * 0.5f;
            s.cpm_acc_ref = 0.0f;
            s.cpm_vel_ref = V;
            s.cpm_pos_ref = s.cpm_start_pos + d_acc + V * (t - T_acc);
        } else if (t <= s.cpm_t_total_s) {
            const float dt = t - T_dcc;
            const float phase = dt / T_acc;
            const float d_acc = V * T_acc * 0.5f;
            const float p_dcc = s.cpm_start_pos + d_acc + V * (T_dcc - T_acc);
            s.cpm_acc_ref = -a_peak * sinf(M_PI * phase);
            s.cpm_vel_ref = V * (1.0f + cosf(M_PI * phase)) * 0.5f;
            s.cpm_pos_ref = p_dcc
                + V * (dt * 0.5f + T_acc * sinf(M_PI * phase) / (2.0f * M_PI));
        } else {
            s.cpm_pos_ref = end;
            s.cpm_vel_ref = 0.0f;
            s.cpm_acc_ref = 0.0f;
            s.last_end = end;
            s.cpm_hold_ts = millis();
            s.cpm_state = CPMState::HOLD;
        }

        // fall through: 이번 tick 은 end 홀드 참조로 제어
    }

    float pos_ref;
    float vel_ref;
    if (s.cpm_state == CPMState::HOLD) {
        pos_ref = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
        vel_ref = 0.0f;
        if (millis() - s.cpm_hold_ts >= CPM_END_HOLD_MS) {
            s.cpm_dir = -s.cpm_dir;
            s.cpm_state = CPMState::BUILD;
        }
    } else {
        pos_ref = s.cpm_pos_ref;
        vel_ref = s.cpm_vel_ref;
    }

    s.imp_q_ref = pos_ref; s.imp_v_ref = vel_ref; s.imp_use_pd = true;   // [33차] 임피던스 참조
    // PD (관절별 검증 게인 — PROM 과 공용 배열)
    float tau_pd = PROM_PD_KP[i] * (pos_ref - s.current_position)
                 + PROM_PD_KD[i] * (vel_ref - s.nl_filt_vel);

    // [24차] stall (0625 안전 4): 추종오차 지속 = 하드스톱/잼/과저항 → 현재 자세 홀드.
    //   guard 어긋남(j1 전원 인가 자세 오류)의 최후 방어선. 타이머는 prom_stall_start_ms
    //   재사용 (CPM/PROM 은 동시 실행 불가, 모드 진입 시 resetDOB 가 0 초기화).
    {
        const float e_pos_cpm = pos_ref - s.current_position;
        if (fabsf(e_pos_cpm) > PROM_STALL_ERR_RAD[i]) {
            if (s.prom_stall_start_ms == 0) {
                s.prom_stall_start_ms = millis();
            } else if (millis() - s.prom_stall_start_ms >= PROM_STALL_ERR_MS) {
                s.g_cpmMode = false;
                s.g_fixMode = true;
                s.fix_target_pos = s.current_position;
                s.current_target_pos = s.current_position;
                s.prom_stall_start_ms = 0;
                Serial.print("[CPM] STALL joint ");
                Serial.print(i);
                Serial.println(" -> hold at current pose (resend cpm to retry)");
                return run_fix_pos(i);
            }
        } else {
            s.prom_stall_start_ms = 0;
        }
    }

    // Stribeck DOB (0611 식, Tf-form) — 관절별 실측 J/마찰
    s.nl_filt_torque = (DOB_Ts * s.nl_tau_cmd + PROM_TF * s.nl_past_filt_torque) / (PROM_TF + DOB_Ts);
    s.nl_est_torque  = (PROM_TF * s.nl_past_est_torque
                        + ST_J[i] * (s.nl_filt_vel - s.nl_past_filt_vel)) / (PROM_TF + DOB_Ts);
    const float tau_st = stribeckFriction(i, s.nl_filt_vel);
    const float disturbance = (s.nl_est_torque - tau_st) - s.nl_filt_torque;
    s.nl_last_dist = disturbance;   // [21차] SD 로깅용

    float tau = tau_pd - ST_DOB_SCALE[i] * (disturbance + tau_st) + tau_g
                + fricFFRef(i, vel_ref) + ditherTorque(i, vel_ref);   // [27차] DOB / [30차] v_ref 마찰 FF + 디더
    if (tau >  CPM_TORQUE_MAX_NM[i]) tau =  CPM_TORQUE_MAX_NM[i];
    if (tau < -CPM_TORQUE_MAX_NM[i]) tau = -CPM_TORQUE_MAX_NM[i];
    tau = clamp_torque(i, tau);

    s.nl_past_filt_vel    = s.nl_filt_vel;
    s.nl_past_filt_torque = s.nl_filt_torque;
    s.nl_past_est_torque  = s.nl_est_torque;
    s.nl_tau_cmd = tau;
    s.target_torque = tau;
    return tau;
}

// ============================================================================
// [Control: PROM / AROM Transparent Mode]
// PROM = 0716 구조: 등속 스윕 위치서보 — 램프 참조(ACC 0.1 rad/s²)를 PD+Stribeck DOB 로
// 추종, 방향은 Jetson speed 부호(dir=반전). 안전층: guard 핀(히스테리시스 재무장),
// stall(관절별 rad/250 ms→홀드), 과속 컷오프, 관절별 토크 클램프. ROM min/max 를 기록·출력.
// ============================================================================
float run_prom(int i) {
    // [jehyeon 2026-07-18 v2] 0716_prom_elbow 구조로 전환:
    //   기준위치 등속 램프 + PD + Stribeck DOB (관절별 실측값) + 0716 안전층 전체.
    //   UDP 의미 유지: prom_velocity_ref = 부호 있는 지령 속도 (Jetson 'dir' = 부호 반전).
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    if (!s.prom_inited) {
        s.prom_ref_pos = s.current_position;   // 현재 절대각에서 램프 시작
        s.prom_vel_ramp = 0.0f;
        s.prom_max_position = s.current_position;
        s.prom_min_position = s.current_position;
        s.nl_filt_vel = 0.0f;    s.nl_past_filt_vel = 0.0f;
        s.nl_filt_torque = 0.0f; s.nl_past_filt_torque = 0.0f;
        s.nl_est_torque = 0.0f;  s.nl_past_est_torque = 0.0f;
        s.nl_tau_cmd = 0.0f;
        s.prom_stall_start_ms = 0;
        s.prom_at_limit = false;
        s.prom_inited = true;
    }

    // 속도 LPF (0716: 1차, Tf = 1/200)
    s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);

    // 누적 ROM 갱신
    if (s.current_position > s.prom_max_position) s.prom_max_position = s.current_position;
    if (s.current_position < s.prom_min_position) s.prom_min_position = s.current_position;

    // ★ 안전(0716-2): 속도 컷오프 — 이상 가속 시 현재 자세 홀드 전환
    if (fabsf(s.nl_filt_vel) > PROM_VEL_CUTOFF_J[i]) {
        s.g_promMode = false;
        s.g_fixMode = true;
        s.fix_target_pos = s.current_position;
        s.current_target_pos = s.current_position;
        Serial.print("[PROM] VEL CUTOFF joint ");
        Serial.print(i);
        Serial.println(" -> hold (resend prom to retry)");
        return run_fix_pos(i);
    }

    // 등속(램프) 기준 생성 (0716) — v_target 부호가 방향, 'dir' 반전도 램프로 부드럽게
    const float v_target = s.prom_velocity_ref;
    const float dv_max = PROM_ACC_RAD_S2 * DOB_Ts;
    float dv = v_target - s.prom_vel_ramp;
    if (dv > dv_max) dv = dv_max;
    else if (dv < -dv_max) dv = -dv_max;
    s.prom_vel_ramp += dv;
    s.prom_ref_pos += s.prom_vel_ramp * DOB_Ts;

    float pos_ref = s.prom_ref_pos;
    float vel_ref = s.prom_vel_ramp;

    // ★ 안전(0716-3): guard 에 기준 클램프 + 적분기 pin (+1회 알림, 위치 히스테리시스 재무장)
    float pr = pos_ref;
    if (pr < PROM_GUARD_MIN_RAD[i]) pr = PROM_GUARD_MIN_RAD[i];
    if (pr > PROM_GUARD_MAX_RAD[i]) pr = PROM_GUARD_MAX_RAD[i];
    if (pr != pos_ref) {
        vel_ref = 0.0f;
        s.prom_vel_ramp = 0.0f;
        if (!s.prom_at_limit) {
            s.prom_at_limit = true;
            Serial.print("[PROM] guard limit joint ");
            Serial.print(i);
            Serial.print(" at ");
            Serial.print(pr / JOINT_GEAR[i] * 180.0f / M_PI, 1);   // [24차] 관절 deg
            Serial.println(" deg -- send 'dir' or 'stop'");
        }
    } else if (s.current_position > PROM_GUARD_MIN_RAD[i] + 0.03f &&
               s.current_position < PROM_GUARD_MAX_RAD[i] - 0.03f) {
        s.prom_at_limit = false;
    }
    pos_ref = pr;
    s.prom_ref_pos = pr;

    // PD (관절별 PROM 게인)
    const float e_pos = pos_ref - s.current_position;
    s.imp_q_ref = pr; s.imp_v_ref = vel_ref; s.imp_use_pd = true;   // [33차] 임피던스 참조
    float tau_pd = PROM_PD_KP[i] * e_pos + PROM_PD_KD[i] * (vel_ref - s.nl_filt_vel);

    // ★ 안전(0716-4): stall — 추종오차(관절별 한계)가 250 ms 지속되면 현재 자세 홀드 전환
    if (fabsf(e_pos) > PROM_STALL_ERR_RAD[i]) {
        if (s.prom_stall_start_ms == 0) {
            s.prom_stall_start_ms = millis();
        } else if (millis() - s.prom_stall_start_ms >= PROM_STALL_ERR_MS) {
            s.g_promMode = false;
            s.g_fixMode = true;
            s.fix_target_pos = s.current_position;
            s.current_target_pos = s.current_position;
            s.prom_stall_start_ms = 0;
            Serial.print("[PROM] STALL joint ");
            Serial.print(i);
            Serial.println(" -> hold at current pose (resend prom to retry)");
            return run_fix_pos(i);
        }
    } else {
        s.prom_stall_start_ms = 0;
    }

    // Stribeck DOB (0611/0716 식, Tf-form) — 관절별 실측 J/마찰
    s.nl_filt_torque = (DOB_Ts * s.nl_tau_cmd + PROM_TF * s.nl_past_filt_torque) / (PROM_TF + DOB_Ts);
    s.nl_est_torque  = (PROM_TF * s.nl_past_est_torque
                        + ST_J[i] * (s.nl_filt_vel - s.nl_past_filt_vel)) / (PROM_TF + DOB_Ts);
    const float tau_st = stribeckFriction(i, s.nl_filt_vel);
    const float disturbance = (s.nl_est_torque - tau_st) - s.nl_filt_torque;
    s.nl_last_dist = disturbance;   // [21차] SD 로깅용

    float tau_g = (p.gc_m != 0.0f) ? (p.gc_m * p.gc_l * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;
    float tau = tau_pd - ST_DOB_SCALE[i] * (disturbance + tau_st) + tau_g
                + fricFFRef(i, vel_ref) + ditherTorque(i, vel_ref);   // [27차] DOB / [30차] v_ref 마찰 FF + 디더

    // ★ 안전(0716-5): PROM 전용 클램프 → 프로필 클램프 → 범위밖 미는 토크 차단
    if (tau >  PROM_TORQUE_MAX_NM[i]) tau =  PROM_TORQUE_MAX_NM[i];
    if (tau < -PROM_TORQUE_MAX_NM[i]) tau = -PROM_TORQUE_MAX_NM[i];
    tau = clamp_torque(i, tau);
    if ((s.current_position >= PROM_GUARD_MAX_RAD[i] && tau > 0.0f) ||
        (s.current_position <= PROM_GUARD_MIN_RAD[i] && tau < 0.0f)) {
        tau = 0.0f;
    }

    s.nl_past_filt_vel    = s.nl_filt_vel;
    s.nl_past_filt_torque = s.nl_filt_torque;
    s.nl_past_est_torque  = s.nl_est_torque;
    s.nl_tau_cmd = tau;
    s.target_torque = tau;
    return tau;
}

// ============================================================================
// [Control: Isometric]
// MOVING: PROM 검증 서보(PD 50/5 + Stribeck DOB)로 목표각까지 램프 접근(18차).
// HOLDING/RESTING: 동일한 고강성 관절별 PD + 구식 DOB로 목표각을 계속 유지한다.
// hold 3회 사이에 3초 휴식을 두고, 완료 후에도 다음 명령까지 목표각을 유지한다.
// ============================================================================
float run_isometric(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);
    s.acceleration = (s.filtered_velocity - s.prev_filtered_velocity) / DOB_Ts;

    const float Kp = ISOMETRIC_KP_J[i];
    const float Kd = ISOMETRIC_KD_J[i];

    // DOB
    s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
    s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
    s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
    s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

    float tau_g = (p.gc_m != 0.0f) ? (p.gc_m * p.gc_l * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;
    float tau_pd = 0.0f;

    // [jehyeon v2] 램프를 HOLDING 에서도 계속 — 도달 판정(±2°) 순간 참조가 램프→최종목표로
    //   점프하며 고강성 Kp × ≤2° 토크 스텝(도달 직전 스파이크)이 생기던 것 제거.
    const float max_step = s.isom_move_speed * DOB_Ts;
    if (s.isom_move_ref < s.isom_target_angle - max_step) s.isom_move_ref += max_step;
    else if (s.isom_move_ref > s.isom_target_angle + max_step) s.isom_move_ref -= max_step;
    else s.isom_move_ref = s.isom_target_angle;

    if (s.isom_state == IsometricState::MOVING_TO_TARGET) {
        // [18차] MOVING 을 PROM/CPM 검증 구조(PROM_PD + Stribeck DOB)로 교체.
        //   17차 고강성 PD + 70% 마찰 FF로도 흔들림 잔존: tanh 스무딩 탓에 실효 FF 가
        //   1.3 Nm 뿐인 데다, 구식 DOB(τ 양성궤환+저역)와 정지마찰이 얽힌 limit cycle.
        //   같은 관절에서 매끈함이 검증된 이동 법칙을 그대로 사용. HOLDING(관절별 고강성 PD +
        //   구식 DOB)은 유지 — 구식 DOB 상태는 함수 상단에서 매 tick 갱신되고 아래서
        //   prev_target_torque 를 인가토크로 채우므로 전환 시 정상상태 토크(중력+마찰)를
        //   그대로 인계받아 스텝이 없다.
        float pos_err = s.isom_target_angle - s.current_position;

        s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);
        s.imp_q_ref = s.isom_move_ref; s.imp_v_ref = 0.0f; s.imp_use_pd = true;   // [33차]
        float tau_mv = PROM_PD_KP[i] * (s.isom_move_ref - s.current_position)
                     + PROM_PD_KD[i] * (0.0f - s.nl_filt_vel);
        s.nl_filt_torque = (DOB_Ts * s.nl_tau_cmd + PROM_TF * s.nl_past_filt_torque) / (PROM_TF + DOB_Ts);
        s.nl_est_torque  = (PROM_TF * s.nl_past_est_torque
                            + ST_J[i] * (s.nl_filt_vel - s.nl_past_filt_vel)) / (PROM_TF + DOB_Ts);
        const float tau_st = stribeckFriction(i, s.nl_filt_vel);
        const float dist_nl = (s.nl_est_torque - tau_st) - s.nl_filt_torque;
        s.nl_last_dist = dist_nl;   // [21차] SD 로깅용
        tau_mv = tau_mv - ST_DOB_SCALE[i] * (dist_nl + tau_st) + tau_g;   // [27차]
        if (tau_mv >  CPM_TORQUE_MAX_NM[i]) tau_mv =  CPM_TORQUE_MAX_NM[i];
        if (tau_mv < -CPM_TORQUE_MAX_NM[i]) tau_mv = -CPM_TORQUE_MAX_NM[i];
        tau_mv = clamp_torque(i, tau_mv);

        s.nl_past_filt_vel    = s.nl_filt_vel;
        s.nl_past_filt_torque = s.nl_filt_torque;
        s.nl_past_est_torque  = s.nl_est_torque;
        s.nl_tau_cmd = tau_mv;

        if (fabs(pos_err) < (2.0f * M_PI / 180.0f) && fabsf(s.nl_filt_vel) < 0.05f) {
            s.isom_rep = 1;
            s.isom_phase_start_ms = millis();
            s.isom_state = IsometricState::HOLDING;
            Serial.print("[ISOM] joint ");
            Serial.print(i);
            Serial.print(" rep 1/");
            Serial.print(ISOMETRIC_TOTAL_REPS);
            Serial.print(" HOLD for ");
            Serial.print(s.isom_hold_duration_ms / 1000.0f, 1);
            Serial.println(" s -- push now");
        }

        s.target_torque = tau_mv;
        s.prev_target_torque = s.target_torque - tau_g;   // 구식 DOB 인계 연속성
        s.prev_filtered_velocity = s.filtered_velocity;
        return s.target_torque;
    } else if (s.isom_state == IsometricState::HOLDING ||
               s.isom_state == IsometricState::RESTING ||
               s.isom_state == IsometricState::COMPLETED) {
        // 운동과 휴식 모두 같은 목표각을 고강성 PD로 유지한다.
        s.imp_q_ref = s.isom_move_ref; s.imp_v_ref = 0.0f; s.imp_use_pd = true;   // [33차]
        tau_pd = Kp * (s.isom_move_ref - s.current_position) - Kd * s.filtered_velocity;

        const uint32_t now_ms = millis();
        if (s.isom_state == IsometricState::HOLDING &&
            now_ms - s.isom_phase_start_ms >= s.isom_hold_duration_ms) {
            if (s.isom_rep < ISOMETRIC_TOTAL_REPS) {
                s.isom_state = IsometricState::RESTING;
                s.isom_phase_start_ms = now_ms;
                Serial.print("[ISOM] joint ");
                Serial.print(i);
                Serial.print(" rep ");
                Serial.print(s.isom_rep);
                Serial.print("/");
                Serial.print(ISOMETRIC_TOTAL_REPS);
                Serial.println(" complete -- REST 3 s");
            } else {
                s.isom_state = IsometricState::COMPLETED;
                s.isom_phase_start_ms = now_ms;
                Serial.print("[ISOM] joint ");
                Serial.print(i);
                Serial.println(" all 3 reps complete -- holding target until next command");
            }
        } else if (s.isom_state == IsometricState::RESTING &&
                   now_ms - s.isom_phase_start_ms >= ISOMETRIC_REST_MS) {
            s.isom_rep++;
            s.isom_state = IsometricState::HOLDING;
            s.isom_phase_start_ms = now_ms;
            Serial.print("[ISOM] joint ");
            Serial.print(i);
            Serial.print(" rep ");
            Serial.print(s.isom_rep);
            Serial.print("/");
            Serial.print(ISOMETRIC_TOTAL_REPS);
            Serial.print(" HOLD for ");
            Serial.print(s.isom_hold_duration_ms / 1000.0f, 1);
            Serial.println(" s -- push now");
        }
    }

    // [jehyeon v2] 관절별 클램프 (elbow 8.3=rated; 원본 0129 는 ±15 명시였음 — 프로필 20 은 과대)
    float tau_isom = tau_pd + tau_g - s.estimated_disturbance;
    if (tau_isom >  CPM_TORQUE_MAX_NM[i]) tau_isom =  CPM_TORQUE_MAX_NM[i];
    if (tau_isom < -CPM_TORQUE_MAX_NM[i]) tau_isom = -CPM_TORQUE_MAX_NM[i];
    s.target_torque = clamp_torque(i, tau_isom);
    s.prev_target_torque = s.target_torque - tau_g;
    s.prev_filtered_velocity = s.filtered_velocity;
    return s.target_torque;
}

// ============================================================================
// [AROM Mode Runner]
// 0522 구조 그대로: τ̂_human(직전 루프 disturbance) → admittance(관절별 M_v/B_v,
// dq/ddq 클램프) → q_ref → impedance PD(70/1.5) → τ_ref = τ_imp + τ_g.
// disturbance = [LPF(J·ω̇) + 저속게이트·70% 운동마찰] − LPF(τ_ref) — ★LPF 입력은 τ_ref
// (τ_cmd 아님): 임피던스 반작용→τ̂_h 복원 채널이 투명감의 핵심. τ_cmd = τ_ref + disturbance.
// ============================================================================
// [40차] 사용자 트리거 PROM 방식 AROM — 추정 없음, 판정만.
//   오차 = 현재각 − 기준각.  |오차| 가 불감대를 넘은 만큼에 비례해 기준을 오차 방향으로 전진.
//   기준은 guard 로 클램프하므로 guard 밖에서는 PD 가 그대로 되돌리는 벽이 된다(DOB 가 없어
//   상쇄되지 않는다). 중력은 피드포워드로 직접 더한다 — 관측기가 없으니 오인할 주체도 없다.
float run_arom_trigger(int i) {
    MotorState& s = m_states[i];
    const float jg = JOINT_GEAR[i];

    if (s.arom_state == AROMState::IDLE) {
        s.arom_init_position = s.current_position;
        s.arom_max_position  = s.current_position;
        s.arom_min_position  = s.current_position;
        s.q_ref_adm          = s.current_position;   // 기준각(홀드 타깃)으로 재사용
        s.dq_ref_adm         = 0.0f;
        s.arom_tau_human     = 0.0f;
        s.arom_state = AROMState::RUNNING;
        Serial.print("[AROM] joint ");
        Serial.print(i);
        Serial.println(" running (trigger-PROM) -- push the limb, reference follows");
    }

    // 속도 LPF (컷오프 판정·PD 감쇠용)
    s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);
    s.nl_past_filt_vel = s.nl_filt_vel;

    if (s.current_position > s.arom_max_position) s.arom_max_position = s.current_position;
    if (s.current_position < s.arom_min_position) s.arom_min_position = s.current_position;

    // 과속 컷오프 (기존 AROM 과 동일 정책, 100 ms 지속)
    if (fabsf(s.nl_filt_vel) > (AROM_VEL_CUTOFF_J[i] > 0.0f ? AROM_VEL_CUTOFF_J[i] : AROM_VEL_CUTOFF)) {
        if (s.prom_stall_start_ms == 0) {
            s.prom_stall_start_ms = millis();
        } else if (millis() - s.prom_stall_start_ms >= 100) {
            s.g_aromMode = false; s.g_fixMode = true;
            s.fix_target_pos = s.current_position; s.current_target_pos = s.current_position;
            s.prom_stall_start_ms = 0;
            Serial.print("[AROM] VEL CUTOFF joint "); Serial.print(i);
            Serial.println(" -> hold (resend arom to retry)");
            return run_fix_pos(i);
        }
    } else {
        s.prom_stall_start_ms = 0;
    }

    // ★기준 전진: 불감대를 넘은 만큼 비례 속도로, 오차 방향으로만.
    const float err = s.current_position - s.q_ref_adm;
    const float excess = fabsf(err) - AROM_TRIG_DEAD[i];
    float v_ref = 0.0f;
    if (excess > 0.0f) {
        v_ref = AROM_TRIG_GAIN[i] * excess;
        if (v_ref > AROM_TRIG_VMAX[i]) v_ref = AROM_TRIG_VMAX[i];
        if (err < 0.0f) v_ref = -v_ref;
        s.q_ref_adm += v_ref * DOB_Ts;
    }
    s.dq_ref_adm = v_ref;
    if (s.q_ref_adm < PROM_GUARD_MIN_RAD[i]) { s.q_ref_adm = PROM_GUARD_MIN_RAD[i]; s.dq_ref_adm = 0.0f; }
    if (s.q_ref_adm > PROM_GUARD_MAX_RAD[i]) { s.q_ref_adm = PROM_GUARD_MAX_RAD[i]; s.dq_ref_adm = 0.0f; }

    // 중력 피드포워드 (관절 프레임 M·L·g·sin(q+φ) → 출력축 /jg)
    float tau_g = 0.0f;
    if (AROM_GC_ML[i] > 0.0f) {
        tau_g = AROM_GC_ML[i] * gc_G * sinf(s.current_position / jg + AROM_GC_PHASE[i]) / jg;
    }

    // [40차-b] 운동마찰 FF (속도 방향, 저속 게이트)
    float tau_f = 0.0f;
    if (AROM_TRIG_FRIC[i] > 0.0f) {
        float g = (fabsf(s.nl_filt_vel) - AROM_FRIC_V_ON) / (AROM_FRIC_V_FULL - AROM_FRIC_V_ON);
        if (g < 0.0f) g = 0.0f;
        if (g > 1.0f) g = 1.0f;
        tau_f = AROM_TRIG_FRIC[i] * g * ST_FC[i] * tanhf(s.nl_filt_vel / 0.08f);
    }

    float tau = PROM_PD_KP[i] * (s.q_ref_adm - s.current_position)
              + PROM_PD_KD[i] * (s.dq_ref_adm - s.nl_filt_vel) + tau_g + tau_f;
    if (tau >  PROM_TORQUE_MAX_NM[i]) tau =  PROM_TORQUE_MAX_NM[i];
    if (tau < -PROM_TORQUE_MAX_NM[i]) tau = -PROM_TORQUE_MAX_NM[i];
    tau = clamp_torque(i, tau);

    s.arom_tau_human = 0.0f;        // 추정하지 않음 (로깅 일관성)
    s.nl_tau_cmd = tau;
    s.target_torque = tau;
    s.imp_use_pd = false;
    return tau;
}

float run_arom(int i) {
    if (AROM_TRIG_J[i]) return run_arom_trigger(i);   // [40차] j1
    m_states[i].imp_use_pd = false;   // [33차] 토크 직결 (kp=kd=0)
    // [jehyeon 2026-07-18 v3] 0522_ad_imp_arom 을 순서까지 그대로 복원 (4차 수정):
    //   Step2: τ̂_human = "직전 루프" disturbance
    //   Step3: Admittance  M_v·q̈_ref + B_v·q̇ = τ̂_h  (감쇠항 = 실제 속도)
    //   Step4: τ_imp = Kp(q_ref−q) + Kd(q̇_ref−q̇)
    //   Step6: τ_ref = τ_imp + τ_g
    //   Step7: disturbance = [LPF(J·ω̇) + gate·마찰(v)] − LPF(τ_ref) ,  τ_cmd = τ_ref + disturbance
    //   ★핵심(3차까지의 버그): LPF 입력은 τ_ref — 최종 τ_cmd 가 아님! τ_cmd 를 넣으면
    //   어시스트가 자기상쇄되고, "임피던스 반작용 → τ̂_h 복원" 채널(0522 투명감의 핵심,
    //   push 순간의 감지 경로)이 끊겨 매우 뻣뻣해짐 — 실기에서 확인.
    //   0522 대비 개선 유지: 마찰모델 Stribeck 실측(+저속 게이트, 드리프트 방지), 안전층 4종.
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    if (s.arom_state == AROMState::IDLE) {
        s.arom_init_position = s.current_position;
        s.arom_max_position = s.current_position;
        s.arom_min_position = s.current_position;
        s.q_ref_adm = s.current_position;
        s.dq_ref_adm = 0.0f;
        s.arom_tau_human = 0.0f;
        s.arom_state = AROMState::RUNNING;
        Serial.print("[AROM] joint ");
        Serial.print(i);
        Serial.println(" running (0522 adm+imp) -- move the limb, robot follows");
    }

    // Step 1: 속도 LPF (0522 동일 Tf=1/200; 속도원은 모터 레지스터 — 0522 는 위치차분이었음)
    s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);

    // 누적 ROM 갱신
    if (s.current_position > s.arom_max_position) s.arom_max_position = s.current_position;
    if (s.current_position < s.arom_min_position) s.arom_min_position = s.current_position;

    // ★ 속도 컷오프 — 100 ms 지속 조건 (7차): 위치차분 속도의 1-tick 스파이크 오탐 제거.
    //   (자가가속은 6차의 70% 운동마찰 보상으로 구조적으로 불가 — 이 컷오프는 최후 방어선)
    //   타이머는 prom_stall_start_ms 재사용 (AROM 에서 미사용 필드).
    if (fabsf(s.nl_filt_vel) > (AROM_VEL_CUTOFF_J[i] > 0.0f ? AROM_VEL_CUTOFF_J[i] : AROM_VEL_CUTOFF)) {
        if (s.prom_stall_start_ms == 0) {
            s.prom_stall_start_ms = millis();
        } else if (millis() - s.prom_stall_start_ms >= 100) {
            s.g_aromMode = false;
            s.g_fixMode = true;
            s.fix_target_pos = s.current_position;
            s.current_target_pos = s.current_position;
            s.prom_stall_start_ms = 0;
            Serial.print("[AROM] VEL CUTOFF joint ");
            Serial.print(i);
            Serial.println(" -> hold (resend arom to retry)");
            return run_fix_pos(i);
        }
    } else {
        s.prom_stall_start_ms = 0;
    }

    // Step 2: τ̂_human = 직전 루프 disturbance (0522 규약)
    const float tau_human = s.arom_tau_human;

    // Step 3: Admittance — 가상 궤적 생성 (감쇠항에 실제 속도)
    float ddq_ref = (tau_human - AROM_B_V_J[i] * s.nl_filt_vel) / AROM_M_V_J[i];
    if (ddq_ref >  AROM_DDQ_MAX) ddq_ref =  AROM_DDQ_MAX;   // [9차] 점프 방지: 궤적 가속 상한
    if (ddq_ref < -AROM_DDQ_MAX) ddq_ref = -AROM_DDQ_MAX;
    s.dq_ref_adm += ddq_ref * DOB_Ts;
    if (s.dq_ref_adm >  AROM_DQ_MAX) s.dq_ref_adm =  AROM_DQ_MAX;
    if (s.dq_ref_adm < -AROM_DQ_MAX) s.dq_ref_adm = -AROM_DQ_MAX;
    s.q_ref_adm += s.dq_ref_adm * DOB_Ts;

    // ★ 가상 궤적을 관절 guard 로 클램프 (windup 방지)
    if (s.q_ref_adm < PROM_GUARD_MIN_RAD[i]) { s.q_ref_adm = PROM_GUARD_MIN_RAD[i]; s.dq_ref_adm = 0.0f; }
    if (s.q_ref_adm > PROM_GUARD_MAX_RAD[i]) { s.q_ref_adm = PROM_GUARD_MAX_RAD[i]; s.dq_ref_adm = 0.0f; }

    // Step 4: Impedance
    const float tau_imp = AROM_KP_IMP_J[i] * (s.q_ref_adm - s.current_position)
                        + AROM_KD_IMP_J[i] * (s.dq_ref_adm - s.nl_filt_vel);

    // Step 5-6: τ_ref = τ_imp + τ_g
    // [39차-b] AROM 전용 중력이 지정된 관절은 그것을 쓴다. 관절 프레임 M·L·g·sin(q_joint + φ) 를
    //   출력축 토크로 환산(/jg) — j1 만 jg=4 이고 나머지는 jg=1 이라 식이 종전과 동일하다.
    const float jg_a = JOINT_GEAR[i];
    float tau_g_in = 0.0f, tau_g_ff = 0.0f;
    if (AROM_GC_ML[i] > 0.0f) {
        // [39차-d] 중력은 **관측기 쪽**에 넣는다 (아래 disturbance). 자리를 두 번 틀렸다:
        //   ① τ_ref 안 → LPF(τ_ref) 로 DC 상쇄, 아무 일도 안 함 (10-07 064910: 팔이 −29.9°까지 끌림)
        //   ② τ_cmd 에 직접 가산(DOB 밖) → 관측기가 FF 가 만든 움직임을 사람 토크로 오인,
        //      τ̂_h 가 상한 +12 에 붙박인 채 +61.7° 까지 폭주 (10-07 065540)
        //   ③ 올바른 자리: disturbance = (관성 + 마찰) − LPF(τ_ref) 는 유도상 **τ_human + τ_중력**
        //      이므로 거기서 중력을 빼면 τ̂_h 가 순수 사람 토크가 되고, τ_cmd = τ_ref + disturbance
        //      가 저절로 중력 보상값이 된다 (정지 시 τ̂_h → 0 으로 수렴, q_ref 이동 ~0.05°).
        tau_g_ff = AROM_GC_ML[i] * gc_G * sinf(s.current_position / jg_a + AROM_GC_PHASE[i]) / jg_a;
    } else {
        tau_g_in = (p.gc_m != 0.0f) ? (p.gc_m * p.gc_l * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;
    }
    const float tau_ref = tau_imp + tau_g_in;

    // Step 7: DOB — ★LPF 입력은 τ_ref (τ_cmd 아님, 0522 동일)
    s.nl_filt_torque = (DOB_Ts * tau_ref + PROM_TF * s.nl_past_filt_torque) / (PROM_TF + DOB_Ts);
    s.nl_est_torque  = (PROM_TF * s.nl_past_est_torque
                        + AROM_DOB_J[i] * (s.nl_filt_vel - s.nl_past_filt_vel)
                        + AROM_DOB_B[i] * DOB_Ts * s.nl_filt_vel) / (PROM_TF + DOB_Ts);
    // 운동마찰만 (정지항 Fs 제외 — 출발 순간 과잉 어시스트 방지) × 70% × 저속 게이트
    const float tau_fric_kin = ST_FC[i] * tanhf(s.nl_filt_vel / 0.08f) + ST_FV[i] * s.nl_filt_vel;
    float fric_gate = (fabsf(s.nl_filt_vel) - AROM_FRIC_V_ON) / (AROM_FRIC_V_FULL - AROM_FRIC_V_ON);
    if (fric_gate < 0.0f) fric_gate = 0.0f;
    if (fric_gate > 1.0f) fric_gate = 1.0f;
    const float disturbance = (s.nl_est_torque + AROM_FRIC_SCALE * fric_gate * tau_fric_kin)
                              - s.nl_filt_torque + tau_g_ff;   // [39차-d] 중력을 관측기에서 제거
    // [39차-c] τ̂_h 와인드업 방지: guard 에서 임피던스가 포화하면 disturbance 가 그 크기를 그대로
    //   받아 발산한다(10-07 실기 관절 −292 Nm). 0 이면 비활성 = 종전 거동.
    // [39차-e] ★중력은 **τ_cmd 에는 포함, 어드미턴스 입력에서는 제외**해야 한다.
    //   disturbance 에 중력을 넣어야 τ_cmd = τ_ref + disturbance 가 DC 에서 중력을 보상하지만,
    //   그 값을 그대로 τ̂_h 로 쓰면 "사람이 1.06 Nm 로 밀고 있다" 로 해석된다. M_v=0.005 이라
    //   어드미턴스 이득이 200 → ddq_ref 가 즉시 상한(15 rad/s²)에 걸려 기준 궤적이 폭주했다
    //   (10-07 070025: τ̂_h 가 t=0 에 +1.125 로 시작 → 300 ms 에 상한 +12 고착 → +36° 까지 주행).
    float tau_h_next = disturbance - tau_g_ff;
    if (AROM_TAU_H_MAX[i] > 0.0f) {
        if (tau_h_next >  AROM_TAU_H_MAX[i]) tau_h_next =  AROM_TAU_H_MAX[i];
        if (tau_h_next < -AROM_TAU_H_MAX[i]) tau_h_next = -AROM_TAU_H_MAX[i];
    }
    s.arom_tau_human = tau_h_next;   // 다음 루프의 Step 2 에서 사용 (0522 last_disturbance)

    float tau = tau_ref + disturbance;   // 0522 AROM 규약: +disturbance

    // [39차-c] guard 벽 — DOB 밖에서 가산 (τ_ref 에 넣으면 상쇄된다).
    float tau_wall_a = 0.0f;
    if (AROM_WALL_KP[i] > 0.0f) {
        const float qmin = PROM_GUARD_MIN_RAD[i], qmax = PROM_GUARD_MAX_RAD[i];
        const float qc = s.current_position;
        if (qc < qmin || qc > qmax) {
            const float qb = (qc < qmin) ? qmin : qmax;
            tau_wall_a = AROM_WALL_KP[i] * (qb - qc) - AROM_WALL_KD[i] * s.nl_filt_vel;
            if (tau_wall_a >  AROM_WALL_TAU_MAX[i]) tau_wall_a =  AROM_WALL_TAU_MAX[i];
            if (tau_wall_a < -AROM_WALL_TAU_MAX[i]) tau_wall_a = -AROM_WALL_TAU_MAX[i];
            if (!s.isot_at_wall) {
                s.isot_at_wall = true;
                Serial.print("[AROM] guard wall joint "); Serial.print(i);
                Serial.print(" at "); Serial.print(qb / JOINT_GEAR[i] * 180.0f / M_PI, 1);
                Serial.println(" deg");
            }
        } else if (qc > qmin + 0.03f && qc < qmax - 0.03f) {
            s.isot_at_wall = false;
        }
    }
    tau += tau_wall_a;   // [39차-d] 벽만 DOB 밖 (중력은 관측기 쪽으로 옮김)

    // ★ 전용 클램프 (사람 상호작용 모드) → 프로필 클램프
    if (tau >  PROM_TORQUE_MAX_NM[i]) tau =  PROM_TORQUE_MAX_NM[i];
    if (tau < -PROM_TORQUE_MAX_NM[i]) tau = -PROM_TORQUE_MAX_NM[i];
    tau = clamp_torque(i, tau);

    s.nl_past_filt_vel    = s.nl_filt_vel;
    s.nl_past_filt_torque = s.nl_filt_torque;
    s.nl_past_est_torque  = s.nl_est_torque;
    s.nl_tau_cmd = tau;
    s.target_torque = tau;
    return tau;
}

// ============================================================================
// [Control: ISOTONIC] (Gravity-emulated free weight)
// 0528 물리 모델: 시작자세(연직, posture 보장)에서 바로 부하 활성(15차) —
// τ = −M·g·L_arm·sin(q) 가상 덤벨 + τ_g, soft-start 1 s. DOB/마찰보상 없음(환자가
// 기계 특성 포함해 느낌 = 실제 아령). [36차] guard 밖 = 토크 0 → guard 로 되돌리는 소프트 월.
// ============================================================================
float run_isotonic(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);

    float tau_g = (p.gc_m != 0.0f) ? (p.gc_m * p.gc_l * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;

    // [36차] Safety Guard: (구) 범위 밖 = 토크 0 → (신) guard 로 되돌리는 소프트 월.
    //   경계 밖으로 나간 양에 비례하는 복원 토크를 계산해 두고, 아래에서 덤벨·중력 항에 더한다.
    //   경계에서 tau_wall = 0 이므로 토크가 연속이고, 벽은 항상 허용범위 안쪽을 향한다.
    float tau_wall = 0.0f;
    {
        const float qmin = PROM_GUARD_MIN_RAD[i], qmax = PROM_GUARD_MAX_RAD[i];
        const float q = s.current_position;
        const bool out = (q < qmin) || (q > qmax);
        if (out) {
            const float qb = (q < qmin) ? qmin : qmax;      // 가장 가까운 guard 경계
            tau_wall = ISOT_WALL_KP[i] * (qb - q) - ISOT_WALL_KD[i] * s.filtered_velocity;
            if (tau_wall >  ISOT_WALL_TAU_MAX[i]) tau_wall =  ISOT_WALL_TAU_MAX[i];
            if (tau_wall < -ISOT_WALL_TAU_MAX[i]) tau_wall = -ISOT_WALL_TAU_MAX[i];
            if (!s.isot_at_wall) {
                s.isot_at_wall = true;
                Serial.print("[ISOT] soft wall joint "); Serial.print(i);
                Serial.print(" at "); Serial.print(qb / JOINT_GEAR[i] * 180.0f / M_PI, 1);
                Serial.println(" deg -- guard 밖, 복원 토크 인가");
            }
        } else if (q > qmin + 0.03f && q < qmax - 0.03f) {
            s.isot_at_wall = false;                         // 히스테리시스 1.7° 안쪽에서 재무장
        }
    }

    // 1. 가상 덤벨 부하 토크 (tau_load = -M * g * L * sin(q))
    float M = s.isotonic_resistance; // kg 단위
    if (s.isotonic_state == IsotonicState::MOVING_TO_TARGET) {
        // [jehyeon v2] 고정목표 순수 PD → 램프 참조 + 관절별 PD + Stribeck DOB.
        //   (구식은 마찰 정지오차가 완료조건 2° 와 경계/초과 — elbow 2.2°, AK80-64 4~8° —
        //    posture 무한대기와 같은 계열의 완료 불가 버그였음)
        const float step = POSTURE_SPEED_RAD_S * DOB_Ts;
        if (s.isotonic_move_ref < s.isotonic_target_angle - step) s.isotonic_move_ref += step;
        else if (s.isotonic_move_ref > s.isotonic_target_angle + step) s.isotonic_move_ref -= step;
        else s.isotonic_move_ref = s.isotonic_target_angle;
        s.imp_q_ref = s.isotonic_move_ref; s.imp_v_ref = 0.0f; s.imp_use_pd = true;   // [33차]

        s.nl_filt_vel = (DOB_Ts * s.current_velocity + PROM_TF * s.nl_past_filt_vel) / (PROM_TF + DOB_Ts);
        float tau_pd = PROM_PD_KP[i] * (s.isotonic_move_ref - s.current_position)
                     + PROM_PD_KD[i] * (0.0f - s.nl_filt_vel);
        s.nl_filt_torque = (DOB_Ts * s.nl_tau_cmd + PROM_TF * s.nl_past_filt_torque) / (PROM_TF + DOB_Ts);
        s.nl_est_torque  = (PROM_TF * s.nl_past_est_torque
                            + ST_J[i] * (s.nl_filt_vel - s.nl_past_filt_vel)) / (PROM_TF + DOB_Ts);
        const float tau_st = stribeckFriction(i, s.nl_filt_vel);
        const float disturbance = (s.nl_est_torque - tau_st) - s.nl_filt_torque;
        s.nl_last_dist = disturbance;   // [21차] SD 로깅용
        float tau_mv = tau_pd - ST_DOB_SCALE[i] * (disturbance + tau_st) + tau_g;   // [27차]
        if (tau_mv >  CPM_TORQUE_MAX_NM[i]) tau_mv =  CPM_TORQUE_MAX_NM[i];
        if (tau_mv < -CPM_TORQUE_MAX_NM[i]) tau_mv = -CPM_TORQUE_MAX_NM[i];
        tau_mv = clamp_torque(i, tau_mv);

        s.nl_past_filt_vel    = s.nl_filt_vel;
        s.nl_past_filt_torque = s.nl_filt_torque;
        s.nl_past_est_torque  = s.nl_est_torque;
        s.nl_tau_cmd = tau_mv;

        const float pos_err = s.isotonic_target_angle - s.current_position;
        if (fabsf(pos_err) < (2.0f * M_PI / 180.0f) && fabsf(s.nl_filt_vel) < 0.05f) {
            s.isotonic_state = IsotonicState::ACTIVE;
            Serial.print("[ISOT] joint ");
            Serial.print(i);
            Serial.println(" reached target -> dumbbell load active (soft-start 1 s)");
        }
        s.target_torque = tau_mv;
        s.prev_filtered_velocity = s.filtered_velocity;
        return s.target_torque;
    }
    s.imp_use_pd = false;   // [33차] 토크 직결 (kp=kd=0)
    // [38차] 프레임 일반화: 내부 상태는 j1 만 출력축(관절×4) 프레임이다. 가상 덤벨은 **관절**
    //   물리량이므로 sin 인자를 관절각으로 되돌리고(/jg), 결과 토크를 출력축으로 환산한다(/jg).
    //   jg=1 인 j0·j2·j3 에서는 종전 식과 완전히 동일하다.
    //   (종전 식은 j1 에서 sin(4·q_joint) 이 되어 관절 45° 에서 부하가 0, 그 너머로는 부호가
    //    뒤집혔고 크기도 4배였다 — 아령이 아니라 4배 주파수 사인파였다.)
    const float jg_d = JOINT_GEAR[i];
    float tau_dumbbell = -M * gc_G * p.dumbbell_l * sinf(s.current_position / jg_d) / jg_d;

    // 3. 최종 인가 토크 (DOB 보상 없이 순수 토크 인가, 0528/0611_isotonic USE_DOB=false 반영)
    // 마찰/관성 보상 없이 환자가 모터의 기계적 특성을 포함해 느끼게 됩니다.
    s.isotonic_soft_gain += DOB_Ts;
    if (s.isotonic_soft_gain > 1.0f) s.isotonic_soft_gain = 1.0f;
    float target_torque = (tau_dumbbell + tau_g) * s.isotonic_soft_gain + tau_wall;   // [36차] 소프트 월

    // [jehyeon v2] 관절별 클램프 (elbow 8.3=rated — 0611 동일) 후 프로필 클램프
    if (target_torque >  CPM_TORQUE_MAX_NM[i]) target_torque =  CPM_TORQUE_MAX_NM[i];
    if (target_torque < -CPM_TORQUE_MAX_NM[i]) target_torque = -CPM_TORQUE_MAX_NM[i];
    s.target_torque = clamp_torque(i, target_torque);
    
    // DOB 업데이트 (실제 인가하진 않더라도 내부 필터 갱신 유지)
    s.acceleration = (s.filtered_velocity - s.prev_filtered_velocity) / DOB_Ts;
    s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
    s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
    s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
    s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

    s.prev_target_torque = s.target_torque - tau_g;
    s.prev_filtered_velocity = s.filtered_velocity;

    return s.target_torque;
}

// ============================================================================
// [Control: FIX_POS] — 부드러운 위치 제어 (0.05 rad/s 속도 제한 + 외란보상)
// 순수 PD 위치 홀드(마찰/DOB 보상 없음) + 참조 램프(평시 0.05, posture 전환 0.15 rad/s).
// stop 후 홀드·posture 전환·isometric 종료 홀드 공용. 토크 캡: 평시 프로필 40%,
// posture 전환 중엔 프로필 전체. 마찰 탓 정지오차 존재 → posture 허용오차 0.10 rad.
// ============================================================================
float run_fix_pos(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);

    // (1) 500 Hz마다 0.0001 rad씩 이동해 참조 속도를 정확히 0.05 rad/s로 제한
    // [24차] ×JOINT_GEAR: 속도 의미를 관절 프레임으로 통일 (j1 은 내부가 출력축이라 ×4 —
    //   아니면 j1 posture 이동만 관절 기준 1/4 속도가 됨)
    float max_step = (posture_ready ? POSTURE_SPEED_RAD_S
                                    : POSTURE_TRANSITION_SPEED_RAD_S) * DOB_Ts * JOINT_GEAR[i];  // [14차]
    if (s.current_target_pos < s.fix_target_pos - max_step) {
        s.current_target_pos += max_step;
    } else if (s.current_target_pos > s.fix_target_pos + max_step) {
        s.current_target_pos -= max_step;
    } else {
        s.current_target_pos = s.fix_target_pos;
    }
    
    // (2) 부드러운 PD 제어 — 모터별 프로필 기반 게인으로 발산 방지
    //     목표 속도가 최대 0.05 rad/s로 제한되므로 토크가 종 캡 이상으로 트일 일이 없음
    float pos_err = s.current_target_pos - s.current_position;
    s.imp_q_ref = s.current_target_pos; s.imp_v_ref = 0.0f; s.imp_use_pd = true;   // [33차]
    float tau_pd = p.cpm_kp * pos_err - p.cpm_kd * s.filtered_velocity;
    
    // 일반 고정은 프로필 토크의 40% 범위, 시작자세 이동은 프로필 전체 제한을 사용합니다.
    float max_fix_tau = p.max_torque * FIX_TORQUE_RATIO;
    if (max_fix_tau < FIX_TORQUE_MIN_NM) max_fix_tau = FIX_TORQUE_MIN_NM;
    if (max_fix_tau > FIX_TORQUE_MAX_NM) max_fix_tau = FIX_TORQUE_MAX_NM;
    if (max_fix_tau > p.max_torque) max_fix_tau = p.max_torque;
    if (!posture_ready) max_fix_tau = p.max_torque;
    if (tau_pd > max_fix_tau) tau_pd = max_fix_tau;
    if (tau_pd < -max_fix_tau) tau_pd = -max_fix_tau;
    
    // (3) 중력 보상
    const float ml = (FIX_GC_ML[i] > 0.0f) ? FIX_GC_ML[i] : (p.gc_m * p.gc_l);   // [31차] FIX 전용 중력값
    float tau_g = (ml != 0.0f) ? (ml * gc_G * sinf(s.current_position + GC_PHASE_RAD[i])) : 0.0f;
    s.target_torque = clamp_torque(i, tau_pd + tau_g);
    return s.target_torque;
}

// ============================================================================
// [SD BIN Logging] — [21차]
// Teensy(0611/0716) SdFat 링버퍼 패턴의 Portenta 이식: SDMMC(FATFS) + SDRAM 4 MB
// 링버퍼(32768행 ≈ 65 s @500 Hz). 제어 tick 은 SDRAM append 만(무차단), SD 쓰기는
// 제어 tick "사이" 유휴 pass 에서 2 KB 상한으로 드레인 → write 지연이 제어에 안 낌.
// 파일은 모드 명령 수신 시 자동 생성(LOGxxx.BIN), stop/E-stop/타임아웃 시 자동 닫힘.
// 디코드: M7/bin2csv.py (LogRow 128 B — 구조를 바꾸면 그 스크립트도 함께 갱신할 것).
// ============================================================================
SDMMCBlockDevice sd_block;
mbed::FATFileSystem sd_fs("sd");
static bool sd_ready = false;

struct __attribute__((packed)) LogRow {
    uint32_t t_us;                    // 로그 시작 기준 경과 [us]
    uint32_t dt_us;                   // 제어 tick 실측 간격 [us] (2000 초과 = 지연 tick)
    uint8_t  mode[JOINT_COUNT];       // 관절별 현재 모드 (ControlMode 값)
    uint8_t  online_bits;             // bit i = joint i online
    uint8_t  posture_ready_u8;
    int8_t   active_joint;            // 명령 수신 시점의 능동 관절
    uint8_t  spare;
    float pos[JOINT_COUNT];           // 내부 프레임 (★j1 은 6:1 출력축 — 관절각 = /4, 이하 동일)
    float vel[JOINT_COUNT];           // raw (위치차분)
    float filt_vel[JOINT_COUNT];      // 모드별 사용 필터속도 (nl_filt_vel 또는 40 Hz LPF)
    float ref[JOINT_COUNT];           // 모드별 참조 위치
    float dist[JOINT_COUNT];          // 모드별 disturbance (AROM 은 τ̂_human)
    float tau_cmd[JOINT_COUNT];
    float tau_meas[JOINT_COUNT];      // ~167 Hz 라운드로빈 갱신 (반복값 존재)
};
static_assert(sizeof(LogRow) == 128, "LogRow layout");

// [22차] 링버퍼를 SDRAM → 내부 SRAM 정적 배열로 교체. SDRAM(0x60000000)은 이 환경에서
//   malloc 이 hard fault 났고, 고정 주소 사용 시에도 저장 데이터가 워드 단위로 깨짐
//   (실기 LOG000: 짝수 워드만 halfword 중복 패턴으로 손상, 홀수 워드는 정상 — 99.8% 불량)
//   → FMC/캐시 경로 신뢰 불가 판단, 폐기. 1024행 × 128 B = 128 KB ≈ 2 s @500 Hz —
//   SD write 스톨(수백 ms) 흡수엔 충분, 초과 시 overrun 카운트만 (제어 무영향).
static const uint32_t LOG_RB_ROWS = 1024;
static const uint32_t LOG_DRAIN_ROWS = 16;   // 유휴 pass 당 최대 2 KB 쓰기
static LogRow log_rb[LOG_RB_ROWS];
static uint32_t log_head = 0, log_tail = 0;
static FILE* sd_log_file = nullptr;
static char sd_log_name[24] = {0};
static int8_t sd_log_joint = -1;
static uint32_t sd_log_t0_us = 0;
static uint32_t sd_log_rows = 0, sd_log_overrun = 0;

// [23차] UDP 로그 스트리밍 — 같은 링버퍼를 두 소비자(SD 파일 / Jetson UDP)가 각자의
//   tail 로 드레인. 수신기: M7/jetson_log_receiver.py (port 9998, LogRow 원형 저장 →
//   bin2csv/plot_data 그대로 호환). 카드 없이 로깅 가능 = 카드 뽑기 불필요 + fwrite
//   스톨(dt_us 스파이크·tick 병합)도 없음. 카드가 있으면 둘 다 기록 (SD = 백업).
static const uint16_t LOG_STREAM_PORT = 9998;
struct __attribute__((packed)) LogStreamHdr {
    uint32_t magic;     // 0x474C374D = "M7LG"
    uint32_t seq;       // 패킷 일련번호 (수신측 유실 검출)
    uint32_t log_id;    // 세션 번호 (수신측 파일 분리 기준)
    uint8_t  count;     // 이 패킷의 행 수 (0 = 세션 종료 마커)
    uint8_t  row_size;  // sizeof(LogRow) — 수신측 검증용
    uint16_t reserved;
};
static const uint32_t LOG_STREAM_MAGIC = 0x474C374DUL;
static const uint32_t LOG_STREAM_ROWS_PER_PKT = 8;   // 8×128+12 ≈ 1 KB < MTU
static uint32_t log_tail_udp = 0;
static bool     log_udp_active = false;
static uint32_t log_stream_seq = 0, log_open_id = 0, log_udp_rows = 0;
static uint8_t  log_pkt_buf[sizeof(LogStreamHdr) + LOG_STREAM_ROWS_PER_PKT * sizeof(LogRow)];

static inline uint32_t logCountFrom(uint32_t tail) {
    return (log_head >= tail) ? (log_head - tail)
                              : (LOG_RB_ROWS - tail + log_head);
}
static inline uint32_t sdLogCount() { return logCountFrom(log_tail); }

// 모드별로 "지금 의미 있는" 참조/필터속도/외란을 선택 (제어 코드는 저장만, 해석은 여기서)
static float logRefOf(int i) {
    const MotorState& s = m_states[i];
    if (!posture_ready) return s.current_target_pos;
    if (s.g_cpmMode)  return s.cpm_pos_ref;
    if (s.g_promMode) return s.prom_ref_pos;
    if (s.g_aromMode) return s.q_ref_adm;
    if (s.g_isometricMode) return s.isom_move_ref;
    if (s.g_isotonicMode)
        return (s.isotonic_state == IsotonicState::MOVING_TO_TARGET)
               ? s.isotonic_move_ref : s.isotonic_target_angle;   // ACTIVE: 참조 없음 — guard 표기
    return s.current_target_pos;   // FIX/IDLE
}
static float logFiltVelOf(int i) {
    const MotorState& s = m_states[i];
    if (s.g_cpmMode || s.g_promMode || s.g_aromMode) return s.nl_filt_vel;
    if (s.g_isometricMode && s.isom_state == IsometricState::MOVING_TO_TARGET) return s.nl_filt_vel;
    if (s.g_isotonicMode && s.isotonic_state == IsotonicState::MOVING_TO_TARGET) return s.nl_filt_vel;
    return s.filtered_velocity;    // 구식 40 Hz LPF 경로 (isom HOLD/REST / isot ACTIVE / fix)
}
static float logDistOf(int i) {
    const MotorState& s = m_states[i];
    if (s.g_aromMode) return s.arom_tau_human;
    if (s.g_cpmMode || s.g_promMode) return s.nl_last_dist;
    if (s.g_isometricMode)
        return (s.isom_state == IsometricState::MOVING_TO_TARGET)
               ? s.nl_last_dist : s.estimated_disturbance;
    if (s.g_isotonicMode)
        return (s.isotonic_state == IsotonicState::MOVING_TO_TARGET)
               ? s.nl_last_dist : s.estimated_disturbance;
    return s.estimated_disturbance;
}

static bool sdLogOpen() {
    if (!sd_ready || sd_log_file) return false;
    bool found = false;
    for (int n = 0; n < 1000 && !found; ++n) {
        snprintf(sd_log_name, sizeof(sd_log_name), "/sd/LOG%03d.BIN", n);
        struct stat st;
        if (stat(sd_log_name, &st) != 0) found = true;
    }
    if (!found) return false;
    sd_log_file = fopen(sd_log_name, "wb");
    return sd_log_file != nullptr;
}

// [23차] 로그 세션 열기: 링/카운터 리셋 후 SD 파일(카드 있으면) + UDP 스트림(Jetson 연결 시)
static void logSessionOpen(int active_joint) {
    log_open_id++;
    log_head = log_tail = log_tail_udp = 0;
    sd_log_rows = 0; sd_log_overrun = 0; log_udp_rows = 0;
    sd_log_joint = (int8_t)active_joint;
    sd_log_t0_us = micros();
    log_udp_active = jetson_connected;
    if (sd_ready) {
        if (sdLogOpen()) {
            Serial.print("[LOG] BIN logging to: ");
            Serial.println(sd_log_name);
        } else {
            Serial.println("[LOG] SD open failed -- SD logging off for this run");
        }
    }
    if (log_udp_active) {
        Serial.print("[LOG] UDP stream to Jetson:");
        Serial.print(LOG_STREAM_PORT);
        Serial.print(" (log_id ");
        Serial.print(log_open_id);
        Serial.println(")");
    }
}

// 제어 tick 끝에서 호출 — SRAM 링버퍼 append 만 (SD/네트워크 접근 없음, 무차단)
static void sdLogAppendRow(uint32_t interval_us) {
    if (!sd_log_file && !log_udp_active) return;
    uint32_t nh = log_head + 1; if (nh >= LOG_RB_ROWS) nh = 0;
    // 가득참 → drop (치명 아님). 활성 소비자 중 가장 느린 쪽 기준.
    if ((sd_log_file && nh == log_tail) ||
        (log_udp_active && nh == log_tail_udp)) { sd_log_overrun++; return; }
    LogRow& r = log_rb[log_head];
    r.t_us = micros() - sd_log_t0_us;
    r.dt_us = interval_us;
    r.online_bits = 0;
    r.posture_ready_u8 = posture_ready ? 1 : 0;
    r.active_joint = sd_log_joint;
    r.spare = 0;
    for (int i = 0; i < JOINT_COUNT; i++) {
        const MotorState& s = m_states[i];
        r.mode[i] = s.prev_mode;
        if (s.online) r.online_bits |= (uint8_t)(1u << i);
        r.pos[i] = s.current_position;
        r.vel[i] = s.current_velocity;
        r.filt_vel[i] = logFiltVelOf(i);
        r.ref[i] = logRefOf(i);
        r.dist[i] = logDistOf(i);
        r.tau_cmd[i] = s.motor_output_torque;
        r.tau_meas[i] = s.real_torque;
    }
    log_head = nh;
    sd_log_rows++;
}

// 제어 tick 사이 유휴 pass 전용 — 2 KB 상한이라 보통 1 ms 미만에 반환 (카드 GC 제외)
static void sdLogDrainOnce() {
    if (!sd_log_file) return;
    if (sdLogCount() < LOG_DRAIN_ROWS) return;   // 2 KB 모일 때까지 대기 (쓰기 횟수 절약)
    uint32_t contig = (log_head > log_tail) ? (log_head - log_tail) : (LOG_RB_ROWS - log_tail);
    if (contig > LOG_DRAIN_ROWS) contig = LOG_DRAIN_ROWS;
    size_t nw = fwrite(&log_rb[log_tail], sizeof(LogRow), contig, sd_log_file);
    log_tail += (uint32_t)nw; if (log_tail >= LOG_RB_ROWS) log_tail = 0;
    if (nw != contig) {                          // 카드 오류/가득참 — 로깅만 중단, 제어 지속
        Serial.println("[LOG] SD write failed -- logging disabled (control unaffected)");
        fclose(sd_log_file); sd_log_file = nullptr;
    }
}

// [23차] 유휴 pass 당 UDP 1패킷(≤8행) 송신 — lwIP 송신은 큐 복사라 sub-ms (SD 와 달리 스톨 없음)
static void udpLogDrainOnce() {
    if (!log_udp_active || !jetson_connected) return;
    uint32_t pending = logCountFrom(log_tail_udp);
    if (pending == 0) return;
    uint32_t contig = (log_head > log_tail_udp) ? (log_head - log_tail_udp)
                                                : (LOG_RB_ROWS - log_tail_udp);
    uint32_t n = (pending < contig) ? pending : contig;
    if (n > LOG_STREAM_ROWS_PER_PKT) n = LOG_STREAM_ROWS_PER_PKT;
    LogStreamHdr h{LOG_STREAM_MAGIC, log_stream_seq++, log_open_id,
                   (uint8_t)n, (uint8_t)sizeof(LogRow), 0};
    memcpy(log_pkt_buf, &h, sizeof(h));
    memcpy(log_pkt_buf + sizeof(h), &log_rb[log_tail_udp], n * sizeof(LogRow));
    Udp.beginPacket(jetson_ip, LOG_STREAM_PORT);
    Udp.write(log_pkt_buf, sizeof(h) + n * sizeof(LogRow));
    Udp.endPacket();
    log_tail_udp += n; if (log_tail_udp >= LOG_RB_ROWS) log_tail_udp = 0;
    log_udp_rows += n;
}

static void sdLogClose(const char* why) {   // [23차] 세션 종료: SD + UDP 스트림 모두 마감
    if (!sd_log_file && !log_udp_active) return;
    // UDP: 잔여 행 전량 송신 + 종료 마커 (수신측은 마커 or 2 s 무수신으로 파일 마감)
    while (log_udp_active && jetson_connected && logCountFrom(log_tail_udp) > 0) {
        udpLogDrainOnce();
    }
    if (log_udp_active) {
        if (jetson_connected) {
            LogStreamHdr h{LOG_STREAM_MAGIC, log_stream_seq++, log_open_id,
                           0, (uint8_t)sizeof(LogRow), 0};
            Udp.beginPacket(jetson_ip, LOG_STREAM_PORT);
            Udp.write((uint8_t*)&h, sizeof(h));
            Udp.endPacket();
        }
        Serial.print("[LOG] UDP stream closed (log_id ");
        Serial.print(log_open_id);
        Serial.print(") rows=");
        Serial.println(log_udp_rows);
        log_udp_active = false;
    }
    if (!sd_log_file) { sd_log_overrun = 0; return; }
    while (log_head != log_tail) {               // 잔량 드레인 (trickle 정책상 보통 수 KB)
        uint32_t contig = (log_head > log_tail) ? (log_head - log_tail) : (LOG_RB_ROWS - log_tail);
        size_t nw = fwrite(&log_rb[log_tail], sizeof(LogRow), contig, sd_log_file);
        log_tail += (uint32_t)nw; if (log_tail >= LOG_RB_ROWS) log_tail = 0;
        if (nw != contig) break;
    }
    fflush(sd_log_file);
    fclose(sd_log_file);
    sd_log_file = nullptr;
    Serial.print("[LOG] closed "); Serial.print(sd_log_name);
    Serial.print(" ("); Serial.print(why);
    Serial.print(") rows="); Serial.print(sd_log_rows);
    Serial.print(" overrun="); Serial.println(sd_log_overrun);
}

void processUDPCommand(const JetsonToM7_UDP& cmd) {
    // 1. 상태 전환 (전체 Enable / Disable)
    if (cmd.robot_state == 1 && !motor_enabled) {
        static uint32_t last_enable_attempt_ms = 0;
        if (millis() - last_enable_attempt_ms < 1000) {
            last_seq_echo = cmd.seq;
            return;
        }
        last_enable_attempt_ms = millis();

        // [26차] 최초 enable 시 전 관절 영점(0x008C): 현재 자세 = 드라이브 0.
        //   guard 프레임 기준이 "드라이브 전원 인가 자세" → "M7 부팅 후 첫 enable(런치) 자세"
        //   로 바뀜 — 운용 규칙: ros2 launch 직전에 전 관절을 기준자세(어깨 하수 0/내회전 0/
        //   팔꿈치 신전 0)에 둘 것. 모터 전원 인가 자세는 이제 무관.
        //   ★최초 1회만: 타임아웃 후 재-enable 에서 재영점하면 운동 중 멈춘 자세가 0 이 되어
        //   guard 가 통째로 밀림. 드라이버 전원을 재인가했으면 Portenta 도 리셋할 것
        //   (기존 E-stop 절차 동일 — 리셋해야 새 프레임에 맞춰 다시 영점을 잡는다).
        static bool boot_zero_done = false;
        const bool do_boot_zero = !boot_zero_done;

        // [37차] 버스 스캔 — 영점·mode·enable 보다 먼저. 부팅 후 1 회만.
        if (!motor_scan_done) {
            scanMotorBus();
            if (CONNECTED_MOTOR_COUNT == 0) {
                // 한 대도 응답이 없으면 배선·전원 문제다. 래치하지 않고 1 s 뒤 재시도.
                Serial.println("[ERROR] Bus scan found no motor -- check driver power and CAN wiring. Retrying.");
                motor_enabled = false;
                wd_data->motor_comm_failed = 0;
                last_seq_echo = cmd.seq;
                return;
            }
            if (checkModuleConfig() < 0) {
                // 기구적으로 불가능한 조합 = 배선 이상. 래치하지 않고 1 s 뒤 재시도.
                motor_enabled = false;
                wd_data->motor_comm_failed = 0;
                last_seq_echo = cmd.seq;
                return;
            }
            applyModuleConfig();
            motor_scan_done = true;
        }

        bool all_motors_ready = true;
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (!isMotorConnected(i)) {
                m_states[i].online = false;
                m_states[i].current_velocity = 0.0f;
                m_states[i].real_torque = 0.0f;
                m_states[i].motor_output_torque = 0.0f;
                expected_online[i] = false;
                continue;
            }
            motor.setMotorID(motor_profiles[i].id);
            // [26차] 영점은 mode/enable 앞에서 (0x008C 는 enable 불필요 — encoder_test 검증).
            if (do_boot_zero) {
                const bool zero_ack = motor.zero();
                delay(5);
                if (!zero_ack) {
                    Serial.print("[WARN] Motor ID "); Serial.print(motor_profiles[i].id);
                    Serial.println(" zero(0x008C) no ack -- verify Position below is ~0");
                }
            }
            // RAW_TORQUE 모드: 외부(우리) 컨트롤러가 토크를 직접 계산하여 전송
            // Write ACK is optional on some MD80 firmware revisions.
            bool mode_ack;
            if (IMP_JOINT[i]) {
                // [33차] 임피던스 모드: kp/kd·maxTorque·초기 참조(현재 위치, v=0, τ_ff=0)를 먼저 써서 enable 순간 점프 방지
                float q_now = 0.0f;
                for (int a = 0; a < 3 && !motor.getPosition(q_now); a++) delay(2);
                delay(2);
                motor.setImpedanceParams(IMP_KP[i], IMP_KD[i]); delay(2);
                motor.writeRegisterFloat(0x0112, motor_profiles[i].max_torque); delay(2);   // maxTorque
                motor.setTargetPosition(q_now); delay(1); motor.setTargetVelocity(0.0f); delay(1); motor.setTargetTorque(0.0f); delay(2);
                m_states[i].imp_q_ref = q_now; m_states[i].imp_v_ref = 0.0f;
                m_states[i].imp_use_pd = true; m_states[i].imp_pd_gains_on = true;
                mode_ack = motor.setMotionMode(MD80_MODE_IMPEDANCE);
                delay(5);
                // 모드 진입 후에도 한 번 더 (펌웨어에 따라 모드 진입 시 파라미터를 다시 읽는 경우 대비)
                motor.setImpedanceParams(IMP_KP[i], IMP_KD[i]); delay(2);
                // ★ readback 진단: 실제로 써졌는지 확인 (0x0050/0x0051/0x0112/0x0141)
                float rb_kp = -1, rb_kd = -1, rb_tmax = -1; uint32_t rb_mode = 255;
                motor.readRegisterFloat(MD80_REG_IMPEDANCE_KP, rb_kp); delay(1);
                motor.readRegisterFloat(MD80_REG_IMPEDANCE_KD, rb_kd); delay(1);
                motor.readRegisterFloat(0x0112, rb_tmax); delay(1);
                motor.readRegisterU32(MD80_REG_MOTION_MODE_STATUS, rb_mode); rb_mode &= 0xFF;
                Serial.print("[INIT] ID "); Serial.print(motor_profiles[i].id);
                Serial.print(" IMPEDANCE mode kp="); Serial.print(IMP_KP[i]); Serial.print(" kd="); Serial.print(IMP_KD[i]);
                Serial.print(" | readback kp="); Serial.print(rb_kp, 3); Serial.print(" kd="); Serial.print(rb_kd, 3);
                Serial.print(" maxTorque="); Serial.print(rb_tmax, 3); Serial.print(" modeStatus="); Serial.println(rb_mode);
            } else {
                mode_ack = motor.setMotionMode(MD80_MODE_RAW_TORQUE);
            }
            delay(5);
            const bool enable_ack = motor.enable();
            delay(10);
            
            // Enable 직후 현재 위치 읽기
            bool position_ok = false;
            for (int attempt = 0; attempt < 3 && !position_ok; attempt++) {
                position_ok = motor.getPosition(m_states[i].current_position);
                if (!position_ok) delay(2);
            }
            // A matching encoder response proves that this motor is reachable.
            // Do not reject it only because a write ACK was omitted.
            m_states[i].online = position_ok;
            m_states[i].current_velocity = 0.0f;
            // The connected motor list defines which axes must answer during
            // the subsequent 500 Hz asynchronous polling stage.
            expected_online[i] = true;
            if (!m_states[i].online) {
                // Do not block the asynchronous polling loop only because the
                // firmware omitted or delayed a blocking register response.
                Serial.print("[INFO] Initial blocking read unavailable ID ");
                Serial.print(motor_profiles[i].id);
                Serial.print(" ModeAck="); Serial.print(mode_ack ? 1 : 0);
                Serial.print(" EnableAck="); Serial.print(enable_ack ? 1 : 0);
                Serial.print(" PositionRead="); Serial.print(position_ok ? 1 : 0);
                Serial.println("; starting async polling.");
                continue;
            }
            
            // 시작 위치 고정
            Serial.print("[INIT] Motor ID "); Serial.print(motor_profiles[i].id);
            Serial.print(" online Position="); Serial.print(m_states[i].current_position, 4);
            Serial.print(" ModeAck="); Serial.print(mode_ack ? 1 : 0);
            Serial.print(" EnableAck="); Serial.println(enable_ack ? 1 : 0);

            // [26차] 영점 직후이므로 위치는 ~0 이어야 함 (0.10 rad ≈ 5.7°; j1 은 출력축 기준).
            //   크게 벗어나면 zero 미반영 — guard 프레임이 어긋난 채 운전하게 되므로 경고.
            if (do_boot_zero && fabsf(m_states[i].current_position) > 0.10f) {
                Serial.print("[WARN] Motor ID "); Serial.print(motor_profiles[i].id);
                Serial.println(" position not ~0 after zero -- guard frame may be wrong! Reset & retry.");
            }

            // [24차] sector 관절(j1) SW 환산 전제 검증 (0625 이식): MD80 기어비 레지스터
            //   0x0017 이 1/6 이어야 위치가 6:1 출력축 프레임 — 다르면 guard/게인 전부
            //   어긋나므로 경고 (블로킹 read 1회, enable 경로라 무해).
            if (JOINT_GEAR[i] != 1.0f || i == 3) {   // [35차] j3(AK60-6 direct) 도 0x0017 확인: 1/6 이면 드라이브 각도 = 출력축(=관절) 프레임
                float gear_reg = 0.0f;
                if (motor.readRegisterFloat(0x0017, gear_reg)) {
                    Serial.print("[INIT] joint "); Serial.print(i);
                    Serial.print(" gear reg 0x0017="); Serial.print(gear_reg, 5);
                    if (fabsf(gear_reg - (1.0f / 6.0f)) > 0.01f) {
                        Serial.println("  ** WARN: != 1/6 -- 4:1 SW frame assumption broken! **");
                    } else {
                        Serial.println("  (ok, 1/6)");
                    }
                } else {
                    Serial.print("[WARN] joint "); Serial.print(i);
                    Serial.println(" gear reg 0x0017 read failed (frame unverified)");
                }
            }

            m_states[i].fix_target_pos = m_states[i].current_position;
            m_states[i].current_target_pos = m_states[i].current_position;
            m_states[i].prev_raw_position = m_states[i].current_position;
        }
        if (all_motors_ready) {
            motor_enabled = true;
            // [26차] enable 성공 후에만 잠금 — 성공 전 재시도(1 s 간격, 팔 정지 상태)는
            //   재영점해도 무해하고, 성공 후 재-enable(타임아웃 복귀)은 재영점 금지.
            boot_zero_done = true;
            wd_data->motor_comm_failed = 0;
            Serial.println("[INIT] Enable commands sent; asynchronous motor polling started.");
        } else {
            // 초기 통신 handshake 실패는 아직 운전 중 단절이 아니므로 watchdog을
            // latch시키지 않고 모든 축을 안전하게 disable한 뒤 1초 후 재시도합니다.
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (!isMotorConnected(i)) continue;
                motor.setMotorID(motor_profiles[i].id);
                motor.setTargetTorque(0.0f);
                delay(2);
                motor.disable();
                delay(2);
                expected_online[i] = false;
            }
            motor_enabled = false;
            wd_data->motor_comm_failed = 0;
            last_seq_echo = cmd.seq;
            return;
        }
    } else if (cmd.robot_state == 0 && motor_enabled) {
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (!isMotorConnected(i)) continue;
            motor.setMotorID(motor_profiles[i].id);
            motor.setTargetTorque(0.0f);
            delay(2);
            motor.disable();
            delay(2);
        }
        motor_enabled = false;
        sdLogClose("disable");   // [21차]
    }

    last_seq_echo = cmd.seq;
    if (!motor_enabled) return;

    int active_joint = -1;
    for (int i = 0; i < JOINT_COUNT; i++) {
        if (!isActiveMode(cmd.joints[i].mode)) continue;

        // [37차] 스캔에서 발견되지 않은 관절에 들어온 명령. 종전에는 조용히 FIX 로 떨어져
        //   "아무 일도 일어나지 않음" 이었다 — 이제 명령마다 1 회 경고한다.
        if (!isSupportedActiveJoint(i)) {
            static uint32_t warned_absent_id = 0xFFFFFFFFu;
            if (cmd.joints[i].command_id != warned_absent_id) {
                warned_absent_id = cmd.joints[i].command_id;
                Serial.print("[WARN] joint "); Serial.print(i);
                Serial.print(" (ID "); Serial.print(motor_profiles[i].id);
                Serial.println(") not found in bus scan -- command ignored");
            }
            continue;
        }

        // [37차] 시작자세 전제 검사 — 필요한 관절이 빠져 있으면 팔을 그 자세로 만들 수
        //   없으므로 중력방향·회전축 전제가 깨진다. 조용히 진행시키지 않는다.
        const int missing = missingPostureJoint(i);
        if (missing >= 0) {
            static uint32_t warned_posture_id = 0xFFFFFFFFu;
            if (cmd.joints[i].command_id != warned_posture_id) {
                warned_posture_id = cmd.joints[i].command_id;
                Serial.print("[WARN] joint "); Serial.print(i);
                Serial.print(": start posture requires joint "); Serial.print(missing);
                Serial.println(" which is absent -- command ignored");
            }
            continue;
        }

        // [24차] j1 의 AROM/ISOT 거부 — active 선택에서 제외하면 아래 파싱에서 FIX 로
        //   강제되고, 다른 능동 관절이 없으면 stop 과 동일하게 처리된다 (경고 1회).
        if (!isModeSupportedForJoint(i, cmd.joints[i].mode)) {
            static uint32_t warned_cmd_id = 0xFFFFFFFFu;
            if (cmd.joints[i].command_id != warned_cmd_id) {
                warned_cmd_id = cmd.joints[i].command_id;
                Serial.print("[WARN] joint ");
                Serial.print(i);
                Serial.println(": AROM unsupported on sector-gear joint (friction not identified) -- command ignored");
            }
            continue;
        }
        active_joint = i;
        break;
    }

    bool newly_started = false;

    // 2. 각 관절별 모드 파싱 및 독립 변수 세팅
    for (int i = 0; i < JOINT_COUNT; i++) {
        uint8_t mode = cmd.joints[i].mode;
        if ((isActiveMode(mode) && i != active_joint) || mode > CMD_FIX_POS) {
            mode = CMD_FIX_POS; // 미지원/손상 명령은 능동 모드 대신 현재 위치 고정
        }
        // [24차] UDP 명령의 각도·속도는 관절 프레임(rad) → 내부(j1: 6:1 출력축) 프레임 환산.
        //   resistance(kg / hold초)는 프레임 무관이라 환산하지 않는다.
        const float jg = JOINT_GEAR[i];
        MotorState& s = m_states[i];
        
        // 새로운 모드로 변경되었을 때 초기화 수행
        if (s.prev_mode != mode || s.last_command_id != cmd.joints[i].command_id) {
            // [jehyeon v2] AROM 종료 시 측정 ROM 출력
            if (s.prev_mode == CMD_AROM && s.arom_state == AROMState::RUNNING) {
                const float r2d = 180.0f / M_PI / JOINT_GEAR[i];   // [24차] 관절 deg 로 출력
                Serial.print("[AROM] ROM joint ");
                Serial.print(i);
                Serial.print(": [");
                Serial.print(s.arom_min_position * r2d, 2);
                Serial.print(", ");
                Serial.print(s.arom_max_position * r2d, 2);
                Serial.print("] deg (span ");
                Serial.print((s.arom_max_position - s.arom_min_position) * r2d, 2);
                Serial.println(" deg)");
            }
            // [jehyeon v2] PROM 종료 시 측정 ROM 출력 (0716 의 stop 출력 대응)
            if (s.prev_mode == CMD_PROM && s.prom_inited) {
                const float r2d = 180.0f / M_PI / JOINT_GEAR[i];   // [24차] 관절 deg 로 출력
                Serial.print("[PROM] ROM joint ");
                Serial.print(i);
                Serial.print(": [");
                Serial.print(s.prom_min_position * r2d, 2);
                Serial.print(", ");
                Serial.print(s.prom_max_position * r2d, 2);
                Serial.print("] deg (span ");
                Serial.print((s.prom_max_position - s.prom_min_position) * r2d, 2);
                Serial.println(" deg)");
            }
            newly_started = true;
            resetDOB(i);
            s.prev_mode = mode;
            s.last_command_id = cmd.joints[i].command_id;
            
            s.g_cpmMode = (mode == CMD_CPM);
            s.g_promMode = (mode == CMD_PROM);
            s.g_aromMode = (mode == CMD_AROM);
            s.g_isometricMode = (mode == CMD_ISOM);
            s.g_isotonicMode = (mode == CMD_ISOT);
            s.g_fixMode = (mode == CMD_FIX_POS);

            if (!s.g_isometricMode) {
                s.isom_state = IsometricState::IDLE;
                s.isom_rep = 0;
                s.isom_phase_start_ms = 0;
            }

            if (s.g_cpmMode) {
                float p_min = cmd.joints[i].target_pos_min * jg;
                float p_max = cmd.joints[i].target_pos_max * jg;
                if (!isfinite(p_min) || !isfinite(p_max)) {
                    p_min = s.current_position;
                    p_max = s.current_position;
                }
                if (p_min > p_max) {
                    float tmp = p_min; p_min = p_max; p_max = tmp;
                }
                // [jehyeon v2] 엔드포인트를 관절 guard 로 클램프 — 하드스톱 너머 지령 방지
                //   (elbow: -120..+5°. 예: cpm,0,-170 을 보내도 -120° 까지만)
                if (p_min < PROM_GUARD_MIN_RAD[i]) p_min = PROM_GUARD_MIN_RAD[i];
                if (p_max > PROM_GUARD_MAX_RAD[i]) p_max = PROM_GUARD_MAX_RAD[i];
                s.P_MIN = p_min;
                s.P_MAX = p_max;
                float requested_speed = fabsf(cmd.joints[i].speed) * jg;
                s.CPM_V_MAX = (isfinite(requested_speed) && requested_speed >= 0.01f)
                    ? requested_speed : POSTURE_SPEED_RAD_S * jg;   // [24차] 기본값도 관절 프레임 의미
                s.last_end = s.current_position;
                s.cpm_state = CPMState::BUILD;
                // 참고 스케치처럼 첫 CPM 구간은 P_MIN(-방향)으로 이동합니다.
                s.cpm_dir = -1;
            }
            else if (s.g_promMode) {
                const float requested_speed = cmd.joints[i].speed * jg;
                s.prom_velocity_ref = (isfinite(requested_speed) && fabsf(requested_speed) >= 0.001f)
                    ? requested_speed : -PROM_DEFAULT_SPEED_RAD_S * jg;   // [24차] 기본값도 관절 프레임 의미
                s.prom_velocity_integral = 0.0f;
                s.prom_inited = false;
            }
            else if (s.g_isometricMode) {
                s.isom_target_angle = isfinite(cmd.joints[i].target_pos_min)
                    ? cmd.joints[i].target_pos_min * jg : s.current_position;
                // [jehyeon v2] 목표각을 관절 guard 로 클램프 (고강성 PD의 하드스톱 지령 방지)
                if (s.isom_target_angle < PROM_GUARD_MIN_RAD[i]) s.isom_target_angle = PROM_GUARD_MIN_RAD[i];
                if (s.isom_target_angle > PROM_GUARD_MAX_RAD[i]) s.isom_target_angle = PROM_GUARD_MAX_RAD[i];
                float hold_sec = cmd.joints[i].resistance;
                if (!isfinite(hold_sec) || hold_sec < 0.0f) hold_sec = 0.0f;
                s.isom_hold_duration_ms = (uint32_t)(hold_sec * 1000.0f);
                s.isom_move_ref = s.current_position;
                const float requested_speed = fabsf(cmd.joints[i].speed) * jg;
                s.isom_move_speed = (isfinite(requested_speed) && requested_speed >= 0.01f)
                    ? requested_speed : POSTURE_SPEED_RAD_S * jg;   // [24차] 기본값도 관절 프레임 의미
                s.isom_rep = 0;
                s.isom_phase_start_ms = 0;
                s.isom_state = IsometricState::MOVING_TO_TARGET;
            }
            else if (s.g_isotonicMode) {
                float resistance = cmd.joints[i].resistance;
                s.isotonic_resistance = (isfinite(resistance) && resistance >= 0.0f)
                    ? resistance : 0.0f;
                s.isotonic_target_angle = isfinite(cmd.joints[i].target_pos_min)
                    ? cmd.joints[i].target_pos_min * jg : s.current_position;
                // [jehyeon v2] 목표각을 관절 guard 로 클램프 + 램프 참조를 현재각에서 시작
                if (s.isotonic_target_angle < PROM_GUARD_MIN_RAD[i]) s.isotonic_target_angle = PROM_GUARD_MIN_RAD[i];
                if (s.isotonic_target_angle > PROM_GUARD_MAX_RAD[i]) s.isotonic_target_angle = PROM_GUARD_MAX_RAD[i];
                s.isotonic_move_ref = s.current_position;
                s.isotonic_soft_gain = 0.0f;
                // [15차] 0528 방식으로 변경: 사전 이동(MOVING_TO_TARGET) 없이 시작 자세에서
                //   바로 부하 활성. posture 전환이 팔을 0°(연직)로 맞춰주므로 활성 순간
                //   τ_dumbbell = -MgL·sin(0) = 0 → 무토크·무동작, 드는 만큼 부하 증가.
                //   (기존: 목표각 도달 후 활성 → 그 순간 +MgL|sin| 전체가 걸려 팔이 저절로
                //    연직으로 낙하 — 어깨에서 두드러져 발견. target_angle 은 guard 용도만 유지)
                s.isotonic_state = IsotonicState::ACTIVE;
            }
            else if (s.g_aromMode) {
                s.arom_state = AROMState::IDLE;
                s.q_ref_adm = s.current_position; 
                s.dq_ref_adm = 0.0f;
            }
            else if (s.g_fixMode || mode == CMD_IDLE) {
                // IDLE 모드이거나 FIX 모드일 때는 현재 위치를 목표로 삼아 굳건히 고정!
                s.fix_target_pos = s.current_position;
                s.current_target_pos = s.current_position;
            }
        }

        // PROM direction updates are carried by the sign of speed and do not
        // restart the start-posture transition.
        if (mode == CMD_PROM) {
            const float requested_speed = cmd.joints[i].speed * jg;   // [24차] 관절→내부 프레임
            if (isfinite(requested_speed) && fabsf(requested_speed) >= 0.001f) {
                s.prom_velocity_ref = requested_speed;
            }
        }
    }

    // A mode starts in two phases: move every connected joint to the selected
    // part's start posture, then run only the selected joint's requested mode.
    if (newly_started && active_joint >= 0) {
        posture_ready = false;
        posture_active_joint = active_joint;

        for (int i = 0; i < JOINT_COUNT; i++) {
            if (!isMotorConnected(i)) continue;

            MotorState& s = m_states[i];
            s.current_target_pos = s.current_position;
            // [24차] posture 표는 관절 프레임 → 내부 프레임 환산 + guard 클램프
            //   (표 값 오류가 있어도 posture 가 하드스톱 너머를 지령하지 못하게 방어)
            float tgt = getStartPostureRad(active_joint, i) * JOINT_GEAR[i];
            if (tgt < PROM_GUARD_MIN_RAD[i]) tgt = PROM_GUARD_MIN_RAD[i];
            if (tgt > PROM_GUARD_MAX_RAD[i]) tgt = PROM_GUARD_MAX_RAD[i];
            s.fix_target_pos = tgt;
        }
        Serial.print("[EVENT] Moving all connected joints to the start posture for joint ");
        Serial.print(active_joint);
        Serial.println(".");

        // [21차] 모드 명령마다 새 로그 세션 (0716 관례) — [23차] SD + Jetson UDP 스트림
        sdLogClose("new command");
        logSessionOpen(active_joint);
    } else if (active_joint < 0) {
        // FIX/IDLE/stop commands cancel an in-progress posture transition.
        posture_ready = true;
        posture_active_joint = -1;
        sdLogClose("stop");   // [21차] 파일이 열려 있을 때만 동작 (idle 100 Hz 패킷엔 no-op)
    }
}

// ============================================================================
// [Setup]
// ============================================================================
void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LOW);  // Red ON during init
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000));

    Serial.println("=== Portenta H7 Rehab Controller (UDP Multi-Motor) ===");
    // Watchdog Shared Memory 초기화
    wd_data->m7_heartbeat = 0;
    wd_data->motor_comm_failed = 0;
    wd_data->e_stop_triggered = 0;
    bootM4();
    delay(100);
    Serial.println("[INIT] M4 booted");

    // CAN 통신 초기화 (기본적으로 첫 번째 모터 ID 바인딩)
    motor.setMotorID(motor_profiles[0].id);
    if (motor.openCAN()) {
        Serial.println("[INIT] CAN opened (FDCAN 1M/5M BRS)");
    } else {
        Serial.println("[ERROR] CAN init failed!");
        while(1) { delay(100); }
    }
    delay(100);

    // 이더넷 & UDP 초기화
    Serial.println("[INIT] Starting Ethernet...");
    Ethernet.begin(mac, ip);
    
    // Portenta Ethernet 연결 대기
    while (Ethernet.linkStatus() == LinkOFF) { 
        delay(500); 
        Serial.print("."); 
    }
    
    Udp.begin(localPort);
    Serial.print("\n[INIT] UDP Listening on IP: ");
    Serial.print(Ethernet.localIP());
    Serial.print(" Port: ");
    Serial.println(localPort);

    // [21차→22차] SD BIN 로깅 초기화: 내부 SRAM 128 KB 링버퍼 + SDMMC(FATFS) 마운트.
    //   슬롯(캐리어의 SDMMC)/카드가 없으면 mount 실패 → 로깅만 꺼지고 제어는 그대로.
    //   (SDRAM 링버퍼는 malloc hard fault + 저장 데이터 워드 손상으로 폐기 — 22차 참조.
    //    업로드는 flash split "1.5MB M7 + 0.5MB M4", M7=Main Core / M4=co-processor.)
    Serial.println("[INIT] SD mount..."); delay(100);
    int sd_err = sd_fs.mount(&sd_block);
    sd_ready = (sd_err == 0);
    if (sd_ready) {
        Serial.println("[INIT] SD card mounted -- BIN logging armed.");
    } else {
        Serial.print("[INIT] SD logging OFF (rb=");
        Serial.print(log_rb ? "ok" : "fail");
        Serial.print(", mount_err=");
        Serial.print(sd_err);
        Serial.println(")");
    }

    // 500Hz 루프 실행
    t_control.attach(&onControlTick, 1.0f / LOOP_RATE_HZ);

    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);   // Blue ON = ready
    Serial.println("[INIT] Setup Complete. Waiting for Jetson UDP Commands...");
}

// ============================================================================
// [Main Loop — 500Hz FDCAN 제어 + 비동기 UDP 폴링]
// 매 2ms tick: FDCAN 제어 후 UDP 큐를 확인합니다.
// Jetson이 10ms마다 보낸 최신 명령에 즉시 한 번 응답하므로 UDP는 100Hz로 동기화됩니다.
// ============================================================================
void loop() {
    // 500Hz (2ms) 제어 주기 고정
    if (!tick_control) {
        sdLogDrainOnce();    // [21차] SD 쓰기는 제어 tick "사이" 유휴 구간에서만 (2 KB 상한)
        udpLogDrainOnce();   // [23차] Jetson UDP 스트림 (패킷 1개/pass)
        return;
    }
    tick_control = false;

    // 프로파일링 변수
    static uint32_t loop_counter = 0;
    static uint32_t last_hz_print_ms = millis();
    static uint32_t total_read_queue_us = 0;
    static uint32_t total_wait_us = 0;
    static uint32_t total_calc_write_us = 0;

    uint32_t loop_start_us = micros();
    
    // 실제 루프 주기(Interval) 측정
    static uint32_t prev_loop_start_us = 0;
    static uint32_t loop_interval_us = 2000; // 초기값 2000us
    if (prev_loop_start_us > 0) {
        loop_interval_us = loop_start_us - prev_loop_start_us;
    }
    prev_loop_start_us = loop_start_us;

    // Watchdog 데이터 갱신
    wd_data->m7_heartbeat++;

    // E-STOP 상태 체크 (M4에서 발동)
    if (wd_data->e_stop_triggered == 1) {
        if (motor_enabled) {
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (!isMotorConnected(i)) continue;
                motor.setMotorID(motor_profiles[i].id);
                motor.setTargetTorque(0.0f);
                delay(2);
                motor.disable();
                delay(2);
            }
            motor_enabled = false;
            Serial.println("[CRITICAL] E-STOP TRIGGERED BY M4 WATCHDOG! Motors Disabled.");
            sdLogClose("estop");   // [21차]
        }
        return; // 모터 제어 루프 진입 차단
    }

    uint32_t t_start = micros();


    // [추가] 1-Shot 통신을 위해 '이전 루프'에서 계산된 목표 토크를 저장해둘 배열
    // static float prev_cmd_tau[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    // if (motor_enabled) {
    //     bool comm_failure = false;

    //     // --- [핵심 추가] 자세 변환 완료 체크 로직 ---
    //     if (!posture_ready) {
    //         bool all_reached = true;
    //         for (int j = 0; j < 4; j++) {
    //             // 오차 0.02 rad (약 1도) 이내면 도달한 것으로 간주
    //             if (fabs(m_states[j].current_target_pos - m_states[j].fix_target_pos) > 0.02f) {
    //                 all_reached = false;
    //                 break;
    //             }
    //         }
    //         if (all_reached) {
    //             posture_ready = true;
    //             Serial.println("[EVENT] Posture Transition Complete! Starting Active Mode.");
                
    //             // 도달한 자세를 기준으로 AROM, PROM 등이 튀지 않게 새롭게 초기화
    //             for (int j = 0; j < 4; j++) {
    //                 if (m_states[j].g_aromMode) m_states[j].arom_state = AROMState::IDLE;
    //                 if (m_states[j].g_promMode) m_states[j].prom_inited = false;
    //                 if (m_states[j].g_cpmMode) {
    //                     m_states[j].last_end = m_states[j].current_position;
    //                     m_states[j].cpm_state = CPMState::BUILD;
    //                 }
    //             }
    //         }
    //     }
        
    //     for (int i = 0; i < 4; i++) {
    //         uint32_t m_start = micros();
    //         motor.setMotorID(motor_profiles[i].id); 

    //         // [핵심 변경 1] 기존 readMotorState(i) 완전 삭제!
    //         // 대신 1-Shot 함수 호출: 이전 틱의 토크를 명령함과 동시에 현재 상태 수신
    //         MD80State new_state;
    //         MotorState& s = m_states[i];
            
    //         if (motor.sendTorqueAndReadState(prev_cmd_tau[i], new_state)) {
    //             s.current_position = new_state.position;
    //             s.current_velocity = new_state.velocity;
    //             s.real_torque = new_state.torque;
    //             s.online = true;
    //         } else {
    //             s.online = false;
    //         }

    //         if (expected_online[i] && !s.online) comm_failure = true;

    //         // [핵심 변경 2] 제어 알고리즘 연산 (DOB, AROM, CPM 등 그대로 유지)
    //         float cmd_tau = 0.0f;
    //         if (!posture_ready) {
    //             cmd_tau = run_fix_pos(i);
    //         } else {
    //             if (s.g_cpmMode) { cmd_tau = run_cpm(i); } 
    //             // ... (나머지 모드 실행 분기 유지) ...
    //         }

    //         cmd_tau = clamp_torque(i, cmd_tau);

    //         // [핵심 변경 3] 여기서 motor.setTargetTorque(cmd_tau) 전송 삭제!
    //         // 대신 다음 루프의 1-Shot 전송을 위해 배열에 담아둡니다.
    //         prev_cmd_tau[i] = cmd_tau; 
    //         s.motor_output_torque = cmd_tau;

    //         fdcan_rtt_total += (micros() - m_start);
    //     }
        
    //     // 등록된 모터 중 하나라도 통신이 끊겼다면 M4에게 E-Stop 요청
    //     wd_data->motor_comm_failed = comm_failure ? 1 : 0;

    // } else {
    //     wd_data->motor_comm_failed = 0;
    // }
    // (M7.ino loop 안쪽)

    if (motor_enabled) {
        bool comm_failure = false;

        // --- 자세 변환 완료 체크 로직 ---
        if (!posture_ready) {
            bool all_reached = true;
            for (int j = 0; j < JOINT_COUNT; j++) {
                if (!isMotorConnected(j)) continue;
                MotorState& s = m_states[j];
                // [24차] 허용오차는 관절 프레임 의미 유지 — 내부(출력축) 프레임인 j1 은 ×gear
                //   (아니면 j1 만 관절 1.4° 라는 4배 엄격한 조건이 되어 posture 가 안 끝남)
                const float pos_tol = POSTURE_POSITION_TOL_RAD * JOINT_GEAR[j];
                const float vel_tol = POSTURE_VELOCITY_TOL_RAD_S * JOINT_GEAR[j];
                const bool reference_reached =
                    fabs(s.current_target_pos - s.fix_target_pos) <= pos_tol;
                const bool motor_reached =
                    s.online &&
                    fabs(s.current_position - s.fix_target_pos) <= pos_tol &&
                    fabs(s.filtered_velocity) <= vel_tol;
                if (!reference_reached || !motor_reached) {
                    all_reached = false;
                    // [14차] 어떤 관절이 왜 완료를 막는지 1 Hz 가시화 (그간 무출력 대기였음)
                    static uint32_t posture_dbg_ms = 0;
                    if (millis() - posture_dbg_ms >= 1000) {
                        posture_dbg_ms = millis();
                        Serial.print("[POSTURE] waiting joint ");
                        Serial.print(j);
                        Serial.print(" pos_err=");
                        Serial.print(s.current_position - s.fix_target_pos, 3);
                        Serial.print(" ref_err=");
                        Serial.print(s.current_target_pos - s.fix_target_pos, 3);
                        Serial.print(" vel=");
                        Serial.print(s.filtered_velocity, 3);
                        Serial.println(s.online ? "" : " (OFFLINE)");
                    }
                    break;
                }
            }
            if (all_reached) {
                posture_ready = true;
                posture_active_joint = -1;
                Serial.println("[EVENT] Posture Transition Complete! Starting Active Mode.");
                for (int j = 0; j < JOINT_COUNT; j++) {
                    if (!isMotorConnected(j)) continue;
                    if (m_states[j].g_aromMode) m_states[j].arom_state = AROMState::IDLE;
                    if (m_states[j].g_promMode) m_states[j].prom_inited = false;
                    if (m_states[j].g_cpmMode) {
                        m_states[j].last_end = m_states[j].current_position;
                        m_states[j].cpm_state = CPMState::BUILD;
                    }
                }
            }
        }


        static int error_counts[JOINT_COUNT] = {0, 0, 0, 0};

        // ────────────────────────────────────────────────────────────────
        // 연결된 3개 모터 순차 폴링 (500Hz)
        // ────────────────────────────────────────────────────────────────
        static uint32_t motor_can_ids[JOINT_COUNT] = {
            motor_profiles[0].id, motor_profiles[1].id,
            motor_profiles[2].id, motor_profiles[3].id
        };
        float new_positions[JOINT_COUNT] = {0};
        bool got[JOINT_COUNT] = {false, false, false, false};

        // RX FIFO 찌꺼기 제거
        motor.flushRxFifo();

        uint32_t t0 = micros();

        int read_queue_fail_count = 0;
        int got_count = 0;
        int torque_read_queue_fail_count = 0;
        int torque_got_count = 0;
        static int torque_read_joint = 0;
        static uint32_t invalid_torque_count = 0;
        int torque_joint_this_tick = -1;
        float new_torque = 0.0f;
        bool torque_got = false;

        // 안정성이 검증된 8바이트 Position 단일 레지스터 읽기만 사용합니다.
        // Position+Torque 다중 레지스터 요청은 MD80 펌웨어별 응답 형식 차이로
        // 세 모터가 모두 무응답이 되는 회귀가 있어 사용하지 않습니다.
        int last_pos_read_joint = -1;   // [37차] 토크 읽기 충돌 판정용
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (!isMotorConnected(i)) continue;
            if (READ_GAP_US[i]) delayMicroseconds(READ_GAP_US[i]);   // [35차] AK60-6 개체: 자기 프레임 앞 조용한 버스 필요
            if (!motor.sendPosReadAsync(motor_can_ids[i])) read_queue_fail_count++;
            // 패킷을 던진 즉시 해당 모터의 응답만 기다림 (타임아웃 400us)
            got_count += motor.pollPosReads(&motor_can_ids[i], &new_positions[i], &got[i], 1, 400);
            last_pos_read_joint = i;
        }

        // Torque is a separate 8-byte register request. One motor is sampled per
        // 2 ms control tick, so every connected motor is updated at about 166.7 Hz.
        for (int scan = 0; scan < JOINT_COUNT; scan++) {
            int candidate = (torque_read_joint + scan) % JOINT_COUNT;
            if (isMotorConnected(candidate)) {
                torque_joint_this_tick = candidate;
                break;
            }
        }
        if (torque_joint_this_tick >= 0) {
            uint32_t torque_id = motor_can_ids[torque_joint_this_tick];
            // [37차] 토크 요청이 직전 위치 읽기와 같은 드라이브를 겨냥하면 그 드라이브가 방금
            //   보낸 응답의 busy window 안에 도착해 드롭된다.
            //   - 4모터: 마지막 읽기 = j3 → `TauRx=0/1 TauID=20` 간헐 실패로 관측(비치명).
            //   - 손목 제거 시: 마지막 읽기 = j2(ID 120) → 같은 충돌이 위치 읽기 완전 두절로
            //     번져 E-stop (10-07 3모터 구성 2회 재현, 5.4 s / 7.2 s. TEC=REC=0 — 전기
            //     오류가 아니라 드라이브가 응답을 멈춤). 4모터 38.8 s 런은 무사.
            if (torque_joint_this_tick == last_pos_read_joint) delayMicroseconds(TAU_READ_GAP_US);
            if (!motor.sendTorqueReadAsync(torque_id)) {
                torque_read_queue_fail_count++;
            } else {
                torque_got_count = motor.pollTorqueReads(
                    &torque_id, &new_torque, &torque_got, 1, 400);
            }

            if (torque_got) {
                // Preserve the last valid measurement if a malformed frame arrives.
                if (isfinite(new_torque) && fabsf(new_torque) <= 200.0f) {
                    m_states[torque_joint_this_tick].real_torque = new_torque;
                } else {
                    invalid_torque_count++;
                }
            }
            torque_read_joint = (torque_joint_this_tick + 1) % JOINT_COUNT;
        }

        uint32_t t1 = micros();
        uint32_t t2 = t1; // Read 완료 시점

        int write_fail_count = 0;

        for (int i = 0; i < JOINT_COUNT; i++) {
            MotorState& s = m_states[i];

            if (!isMotorConnected(i)) {
                s.online = false;
                s.current_velocity = 0.0f;
                s.real_torque = 0.0f;
                s.motor_output_torque = 0.0f;
                error_counts[i] = 0;
                continue;
            }

            if (got[i]) {
                float pos = new_positions[i];
                const bool first_valid_sample = !s.online;
                if (!first_valid_sample) {
                    // [21차] 위치차분 dt 를 실측 tick 간격으로: SD 드레인 등으로 tick 이 늦으면
                    //   고정 Ts 나눗셈이 속도를 지연 배율만큼 과대평가 (5 ms 지연 → 2.5×) →
                    //   컷오프/댐핑 오작동. 정시 tick(≈2000 us)에서는 기존과 동일.
                    float dt_s = (float)loop_interval_us * 1e-6f;
                    if (dt_s < DOB_Ts) dt_s = DOB_Ts;
                    if (dt_s > 0.1f) dt_s = 0.1f;
                    s.current_velocity = (pos - s.prev_raw_position) / dt_s;
                } else {
                    s.current_velocity = 0.0f;
                }
                s.current_position = pos;
                s.prev_raw_position = pos;
                if (first_valid_sample) {
                    // Hold the measured pose when communication first becomes
                    // valid; never command an uninitialised zero-radian target.
                    s.fix_target_pos = pos;
                    s.current_target_pos = pos;
                    s.filtered_velocity = 0.0f;
                }
                // 측정 토크 레지스터는 모터 펌웨어 간 호환성이 없어 제어루프에서
                // 요청하지 않습니다. 현재 알고리즘은 측정 토크를 feedback에 쓰지 않습니다.
                s.online = true;
                error_counts[i] = 0;
            } else {
                s.online = false;
                error_counts[i]++;
            }

            if (expected_online[i] && error_counts[i] > 50) {
                comm_failure = true;
                if (error_counts[i] == 51) {
                    FDCANDiag fail_diag = motor.getBusDiagnostics();
                    Serial.print("[ERROR] Async Position timeout ID ");
                    Serial.print(motor_can_ids[i]);
                    Serial.print(" BusOff="); Serial.print(fail_diag.busOff ? 1 : 0);
                    Serial.print(" ErrPassive="); Serial.print(fail_diag.errorPassive ? 1 : 0);
                    Serial.print(" TEC="); Serial.print(fail_diag.txErrorCount);
                    Serial.print(" REC="); Serial.println(fail_diag.rxErrorCount);
                }
            }

            // 제어 알고리즘 실행
            float cmd_tau = 0.0f;
            if (!posture_ready) {
                cmd_tau = run_fix_pos(i);
            } else {
                if (s.g_cpmMode)            cmd_tau = run_cpm(i);
                else if (s.g_promMode)      cmd_tau = run_prom(i);
                else if (s.g_aromMode)      cmd_tau = run_arom(i);
                else if (s.g_isometricMode) cmd_tau = run_isometric(i);
                else if (s.g_isotonicMode)  cmd_tau = run_isotonic(i);
                else                        cmd_tau = run_fix_pos(i);
            }

            cmd_tau = clamp_torque(i, cmd_tau);
            s.motor_output_torque = cmd_tau;

            if (IMP_JOINT[i]) {
                // [33차] 임피던스 관절: 모드 전환 시 드라이버 kp/kd 를 참조모드(IMP_KP/KD) ↔ 토크직결(0/0) 로 교체 (blocking 1회)
                if (s.imp_use_pd != s.imp_pd_gains_on) {
                    motor.setMotorID(motor_can_ids[i]);
                    motor.setImpedanceParams(s.imp_use_pd ? IMP_KP[i] : 0.0f, s.imp_use_pd ? IMP_KD[i] : 0.0f);
                    s.imp_pd_gains_on = s.imp_use_pd;
                    Serial.print("[IMP] joint "); Serial.print(i); Serial.println(s.imp_use_pd ? " driver PD on" : " driver PD off (torque)");
                }
                if (s.imp_use_pd) {
                    float tau_ff = clamp_torque(i, gravityFF(i, s.current_position) + fricFFRef(i, s.imp_v_ref));
                    s.motor_output_torque = tau_ff;   // 로그 tau_cmd = τ_ff (드라이버 PD 몫은 tau_meas 에서 확인)
                    if (IMP_WRITE_GAP_US[i]) delayMicroseconds(IMP_WRITE_GAP_US[i]);
                    if (!motor.sendPositionWriteAsync(motor_can_ids[i], s.imp_q_ref)) write_fail_count++;
                    delayMicroseconds(IMP_WRITE_GAP_US[i]);   // 같은 드라이브 연속 write: MD80 busy window(~50 µs) 회피 (관절별)
                    if (!motor.sendVelocityWriteAsync(motor_can_ids[i], s.imp_v_ref)) write_fail_count++;
                    delayMicroseconds(IMP_WRITE_GAP_US[i]);
                    if (!motor.sendTorqueWriteAsync(motor_can_ids[i], tau_ff)) write_fail_count++;
                } else {
                    if (!motor.sendTorqueWriteAsync(motor_can_ids[i], cmd_tau)) write_fail_count++;
                }
            } else {
                // (3) 토크 명령 전송 (RAW_TORQUE 모드)
                if (!motor.sendTorqueWriteAsync(motor_can_ids[i], cmd_tau)) {
                    write_fail_count++;
                }
            }
        }

        uint32_t t3 = micros();

        total_read_queue_us += (t1 - t0);
        total_wait_us += (t2 - t1);
        total_calc_write_us += (t3 - t2);
        loop_counter++;

        wd_data->motor_comm_failed = comm_failure ? 1 : 0;

        sdLogAppendRow(loop_interval_us);   // [21차] tick 당 1행 (SDRAM append, 무차단)

        // [진단 출력] 1초에 한 번 통신 상태 출력
        if (millis() - last_hz_print_ms > 1000) {
            uint32_t dt = millis() - last_hz_print_ms;
            // 최초 실행 시 dt가 비정상적으로 크면 리셋
            if (dt > 2000 && loop_counter < 100) {
                last_hz_print_ms = millis();
                loop_counter = 0;
                total_read_queue_us = 0;
                total_wait_us = 0;
                total_calc_write_us = 0;
            } else if (loop_counter > 0) {
                float actual_hz = (float)loop_counter / (dt / 1000.0f);
                float avg_read_q = (float)total_read_queue_us / loop_counter;
                float avg_wait = (float)total_wait_us / loop_counter;
                float avg_calc = (float)total_calc_write_us / loop_counter;
                FDCANDiag diag = motor.getBusDiagnostics();

                Serial.print("[PERF] Actual Loop Rate: "); Serial.print(actual_hz); Serial.println(" Hz");
                Serial.print("       Avg ReadQ+Poll: "); Serial.print(avg_read_q + avg_wait); Serial.print(" us | ");
                Serial.print("Avg Calc/WriteQ: "); Serial.print(avg_calc); Serial.println(" us");

                Serial.print("[DIAG] gotCount="); Serial.print(got_count); Serial.print("/");
                Serial.print(CONNECTED_MOTOR_COUNT); Serial.print(" | ");
                Serial.print("TxFail_R="); Serial.print(read_queue_fail_count);
                Serial.print(" TauRx="); Serial.print(torque_got_count); Serial.print("/1");
                Serial.print(" TauID=");
                if (torque_joint_this_tick >= 0) Serial.print(motor_can_ids[torque_joint_this_tick]);
                else Serial.print(-1);
                Serial.print(" TauTxFail="); Serial.print(torque_read_queue_fail_count);
                Serial.print(" TauInvalid="); Serial.print(invalid_torque_count);
                Serial.print(" TxFail_W="); Serial.print(write_fail_count);
                Serial.print(" BusOff="); Serial.print(diag.busOff ? 1 : 0);
                Serial.print(" ErrPassive="); Serial.print(diag.errorPassive ? 1 : 0);
                Serial.print(" TEC="); Serial.print(diag.txErrorCount);
                Serial.print(" REC="); Serial.println(diag.rxErrorCount);

                loop_counter = 0;
                total_read_queue_us = 0;
                total_wait_us = 0;
                total_calc_write_us = 0;
                last_hz_print_ms = millis();
            }
        }
    } else {
        wd_data->motor_comm_failed = 0;
    }

    uint32_t fdcan_rtt_total = micros() - loop_start_us;

    // ── UDP 서비스 (비동기 폴링) ──
    // [수정] Jetson이 100Hz(10ms) 간격으로 송신하므로, M7은 타이머 블록 없이
    // 매 루프마다 UDP 패킷을 즉시 확인하고 수신 즉시 응답하여 Jitter를 최소화합니다.
    static uint32_t last_udp_rx_ms = 0;

    int ps = Udp.parsePacket();
    JetsonToM7_UDP cmd;
    bool got_pkt = false;
        while (ps > 0) {
            if (ps == sizeof(JetsonToM7_UDP)) {
                Udp.read((char*)&cmd, sizeof(cmd));
                got_pkt = true;
                jetson_ip = Udp.remoteIP();
                jetson_port = Udp.remotePort();
            } else {
                Udp.flush();
            }
            ps = Udp.parsePacket();
        }

        if (got_pkt) {
            jetson_connected = true;
            last_udp_rx_ms = millis();
            processUDPCommand(cmd);

            // Jetson으로부터 패킷을 받았을 때만 응답을 보냅니다 (100Hz 동기화)
            M7ToJetson_UDP reply;
            reply.robot_state = motor_enabled ? 1 : 0;
            reply.seq_echo = last_seq_echo;
            reply.m7_rx_to_tx_us = loop_interval_us; // 제어 루프 주기 (목표 2000us)
            reply.fdcan_rtt_us = fdcan_rtt_total;    // 통신 소요 시간 (약 650us)
            for (int i = 0; i < JOINT_COUNT; i++) {
                // [24차] 상태는 관절 프레임으로 송신 (j1 sector 4:1): 각도·속도 /gear, 토크 ×gear
                reply.joints[i].pos = m_states[i].current_position / JOINT_GEAR[i];
                reply.joints[i].vel = m_states[i].current_velocity / JOINT_GEAR[i];
                reply.joints[i].tau_meas = m_states[i].real_torque * JOINT_GEAR[i];
                reply.joints[i].tau_cmd = m_states[i].motor_output_torque * JOINT_GEAR[i];
                reply.joints[i].online = (isMotorConnected(i) && m_states[i].online) ? 1 : 0;
                reply.joints[i].isom_phase = static_cast<uint8_t>(m_states[i].isom_state);
                reply.joints[i].isom_rep = m_states[i].isom_rep;
                reply.joints[i].isom_total_reps = ISOMETRIC_TOTAL_REPS;
                reply.joints[i].isom_remaining_ms = getIsometricRemainingMs(m_states[i]);
            }
            Udp.beginPacket(jetson_ip, jetson_port);
            Udp.write((uint8_t*)&reply, sizeof(reply));
            Udp.endPacket();
        }

        if (motor_enabled && last_udp_rx_ms > 0 && (millis() - last_udp_rx_ms > 500)) {
            Serial.println("[SAFETY] Jetson Communication Timeout!");
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (!isMotorConnected(i)) {
                    m_states[i].online = false;
                    m_states[i].motor_output_torque = 0.0f;
                    continue;
                }
                motor.setMotorID(motor_profiles[i].id);
                motor.setTargetTorque(0.0f); delay(5);
                motor.disable(); delay(5);
                m_states[i].prev_mode = CMD_IDLE;
            }
            motor_enabled = false;
            jetson_connected = false;
            sdLogClose("jetson timeout");   // [21차]
        }
} // loop() 끝
