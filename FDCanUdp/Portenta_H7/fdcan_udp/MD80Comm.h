#pragma once
/**
 * MD80Comm.h - MAB Robotics MD80 Motor Driver Communication
 * ============================================================================
 * FDCAN Pipelining Protocol (1kHz Optimized, 5Mbps)
 *
 * [v2 핵심 변경] 8바이트 단일 레지스터 Read 방식으로 전환
 *   기존: 20바이트 멀티-레지스터 Read (pos+vel+torque 동시 요청)
 *         → RX FIFO에 20B × 4개 = 80B 쌓여 오버플로우 → 2-by-2 batch 필요
 *   변경: 8바이트 단일 레지스터 Read (position만)
 *         → RX FIFO에 8B × 4개 = 32B → 4개 한번에 처리 가능!
 *         → velocity는 position 차분으로 소프트웨어 계산
 *
 * [FIX_POS 모드] MD80_MODE_IMPEDANCE 활용
 *   모터 내부 40kHz PD 제어기를 사용하여 위치를 추종하므로,
 *   M7의 1kHz 토크 제어기에서 발생하던 발산 문제가 원천적으로 차단됩니다.
 *
 * [MAB 매뉴얼 참고 - MD/Communication/fdcan.html]
 *   - Frame Type 0x42: WRITE_REGISTER(FD) - 레지스터 쓰기
 *   - Frame Type 0x41: READ_REGISTER(FD) - 단일 레지스터 읽기
 *   - Data Baudrate: 5Mbps (BRS 활성, canBaudrate 레지스터 0x002)
 * ============================================================================
 */

#include <Arduino.h>
#include <Arduino.h>
#include <cstring>

// ─── MD80 Frame Types ───
#define MD80_FRAME_WRITE_REGISTER       0x44
#define MD80_FRAME_READ_REGISTER        0x43
#define MD80_FRAME_WRITE_REGISTER_FD    0x42
#define MD80_FRAME_READ_REGISTER_FD     0x41
#define MD80_FRAME_WRITE_REGISTER_LEGACY 0x40
#define MD80_FRAME_LEGACY_RESPONSE       0x0A

// ─── MD80 Register IDs ───
// [2026-07-06 수정] mabrobotics/MD80-x-CANdle-Documentation의 fdcan.md 원문(register table)을
// 직접 확인하여 아래 4개 레지스터 주소 오류를 정정함:
//   MOTOR_TORQUE:        0x0066 → 0x0064 (문서 register table: motorTorque = 0x064)
//   OUTPUT_ENCODER_POS:   0x0065 → 0x0024 (문서 register table: outputEncoderPosition = 0x024)
//   OUTPUT_ENCODER_VEL:   0x0064 → 0x0023 (문서 register table: outputEncoderVelocity = 0x023)
//   IMPEDANCE_KP/KD:  0x0200/0x0201 → 0x0050/0x0051 (문서 register table: motorImpPidKp/Kd)
// 기존 0x0066 torque 주소는 문서에 존재하지 않는 미정의 레지스터였고, 이게 멀티-레지스터
// READ 프레임(0x41, position+velocity+torque 동시 요청) 전체를 조용히 실패시켜
// 응답 자체가 오지 않게 만든 원인으로 추정됨 (미정의 레지스터 접근 시 드라이브는 아무
// 응답도 만들지 않는다고 문서에 명시되어 있음).
#define MD80_REG_CONTROL_WORD           0x0142
#define MD80_REG_MOTION_MODE_COMMAND    0x0140
#define MD80_REG_MOTION_MODE_STATUS     0x0141
#define MD80_REG_TARGET_POS             0x0150
#define MD80_REG_TARGET_VEL             0x0151
#define MD80_REG_TARGET_TORQUE          0x0152
#define MD80_REG_MAIN_ENCODER_POS       0x0063
#define MD80_REG_MAIN_ENCODER_VEL       0x0062
#define MD80_REG_OUTPUT_ENCODER_POS     0x0024
#define MD80_REG_OUTPUT_ENCODER_VEL     0x0023
#define MD80_REG_TORQUE                 0x0064
#define MD80_REG_ZERO                   0x008C
#define MD80_REG_BLINK                  0x008B
#define MD80_REG_RESET                  0x0088
#define MD80_REG_SAVE                   0x0080
#define MD80_REG_CLEAR_ERRORS           0x008A
#define MD80_REG_IMPEDANCE_KP           0x0050
#define MD80_REG_IMPEDANCE_KD           0x0051
#define MD80_REG_MOSFET_TEMPERATURE     0x0806
#define MD80_REG_FIRMWARE_VERSION       0x0802

// ─── Control Word Values ───
#define MD80_CONTROL_WORD_ENABLE        0x27
#define MD80_CONTROL_WORD_DISABLE       0x40

// ─── Motion Modes ───
enum MD80MotionMode : uint8_t {
    MD80_MODE_IDLE      = 0,
    MD80_MODE_POSITION  = 1,
    MD80_MODE_VELOCITY  = 2,
    MD80_MODE_RAW_TORQUE = 3,
    MD80_MODE_IMPEDANCE = 4,
};

// ─── Motor State ───
struct MD80State {
    float position;     // rad
    float velocity;     // rad/s
    float torque;       // Nm
    bool  valid;
};

// ─── FDCAN 버스 진단 정보 ───
// 통신이 원인불명으로 두절될 때, 프로토콜/레지스터 문제가 아니라 FDCAN 페리페럴 자체가
// Bus-Off/Error-Passive 상태에 빠졌는지를 직접 확인하기 위한 구조체.
struct FDCANDiag {
    bool busOff;
    bool errorPassive;
    bool errorWarning;
    uint8_t txErrorCount;
    uint8_t rxErrorCount;
};

// ─── MD80 Communication Class ───
class MD80Comm {
public:
    MD80Comm();

    void setMotorID(uint32_t id) { motor_id_ = id; }
    uint32_t getMotorID() const { return motor_id_; }

    bool openCAN();

    // ─── Low-level register access (8-byte CAN2.0 compatible) ───
    bool writeRegisterU8(uint16_t regID, uint8_t value);
    bool writeRegisterFloat(uint16_t regID, float value);
    bool readRegisterFloat(uint16_t regID, float& value);
    // [진단용] uint32 레지스터 읽기 (firmwareVersion=0x802, buildDate=0x800 등에 사용)
    bool readRegisterU32(uint16_t regID, uint32_t& value);

    // ─── High-level commands ───
    bool enable();
    bool disable();
    bool zero();
    bool blink();
    bool clearErrors();

    bool setMotionMode(MD80MotionMode mode);
    bool setTargetTorque(float torque_Nm);
    bool setTargetPosition(float pos_rad);
    bool setTargetVelocity(float vel_rads);
    bool setImpedanceParams(float kp, float kd);

    bool getPosition(float& pos_rad);
    bool getVelocity(float& vel_rads);
    bool getTorque(float& torque_Nm);

    // ─── [비고속 경로] Combined: 2-Transaction Write Torque + Read State (Blocking) ───
    // Write(8B, blocking) + Read(20B, blocking) 순차 처리 → 모터 1개당 왕복 2회.
    // 파이프라이닝을 적용하지 않는 단일 모터 테스트/디버그용으로만 사용 권장.
    bool sendTorqueAndReadState(float torque_Nm, MD80State& state);

    // ─── [1kHz 파이프라인] Async Write/Read ───
    void flushRxFifo();
    
    // 8바이트 Write 프레임을 TX FIFO에 큐잉만 하고 즉시 리턴 (응답 대기 없음)
    bool sendTorqueWriteAsync(uint32_t can_id, float torque_Nm);
    bool sendVelocityWriteAsync(uint32_t can_id, float vel_rads);
    bool sendPositionWriteAsync(uint32_t can_id, float pos_rad);
    
    // [v2 핵심] 8바이트 단일 레지스터 Read 요청 (position만)
    // 기존 20B 멀티-레지스터 방식 대비 프레임 크기가 1/2.5로 축소되어
    // 4개 모터의 응답이 RX FIFO에 동시에 수용 가능
    bool sendStateReadAsync(uint32_t can_id);
    int pollStateReads(const uint32_t* motor_ids, MD80State* states, bool* got,
                       int count, uint32_t timeout_us);

    bool sendPosReadAsync(uint32_t can_id);
    int pollPosReads(const uint32_t* motor_ids, float* positions, bool* got,
                     int count, uint32_t timeout_us);

    // Stable 8-byte single-register torque read (register 0x0064).
    bool sendTorqueReadAsync(uint32_t can_id);
    int pollTorqueReads(const uint32_t* motor_ids, float* torques, bool* got,
                        int count, uint32_t timeout_us);

    // [v4 추가] Position과 Torque 2개의 레지스터 동시 요청
    bool sendPosAndTorqueReadAsync(uint32_t can_id);
    int pollPosAndTorqueReads(const uint32_t* motor_ids, float* positions, float* torques, bool* got,
                              int count, uint32_t timeout_us);

    // [진단용] FDCAN 페리페럴이 Bus-Off/Error-Passive 상태인지 직접 확인.
    FDCANDiag getBusDiagnostics();

private:
    uint32_t motor_id_;

    // Core transact: send 8-byte frame (CAN2.0 compatible), wait for reply
    bool transact(uint8_t* txBuf, uint8_t* rxBuf, uint32_t timeout_us = 2000);
    
    // [신규] FDCAN 가변 길이 transact: 최대 64바이트 송수신
    // txLen/rxLen: 실제 데이터 바이트 수 (8, 12, 16, 20, 24, 32, 48, 64 중 하나)
    bool transactFD(uint8_t* txBuf, uint32_t txLen, uint8_t* rxBuf, uint32_t rxLen, uint32_t timeout_us = 500);
};
