#include "MD80Comm.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64]; // CAN FD 확장 대응 (최대 64바이트)

MD80Comm::MD80Comm() : motor_id_(0x64) {}

// ============================================================================
// [FDCAN 초기화]
// Nominal: 1Mbps (CAN2.0 호환), Data: 5Mbps (BRS 활성)
// ============================================================================
bool MD80Comm::openCAN() {
    __HAL_RCC_FDCAN_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE(); 
    __HAL_RCC_GPIOH_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_13; 
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    hfdcan1.Instance = FDCAN1; 
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL; 
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE; 
    hfdcan1.Init.ProtocolException = DISABLE;

    // Nominal Bitrate: 1Mbps
    uint32_t fdcan_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    uint32_t nom_tq = fdcan_clk / 1000000; uint32_t nom_pre = 1;
    while(nom_tq > 120) { nom_pre++; nom_tq = fdcan_clk / (1000000 * nom_pre); }
    hfdcan1.Init.NominalPrescaler = nom_pre; 
    hfdcan1.Init.NominalTimeSeg1 = (nom_tq * 8) / 10 - 1; 
    hfdcan1.Init.NominalTimeSeg2 = nom_tq - 1 - hfdcan1.Init.NominalTimeSeg1; 
    hfdcan1.Init.NominalSyncJumpWidth = 1;

    // Data Bitrate: 5Mbps (BRS 활성 시)
    uint32_t data_tq = fdcan_clk / 5000000; uint32_t data_pre = 1;
    while(data_tq > 24) { data_pre++; data_tq = fdcan_clk / (5000000 * data_pre); }
    hfdcan1.Init.DataPrescaler = data_pre; 
    hfdcan1.Init.DataTimeSeg1 = (data_tq * 8) / 10 - 1;
    hfdcan1.Init.DataTimeSeg2 = data_tq - 1 - hfdcan1.Init.DataTimeSeg1; 
    hfdcan1.Init.DataSyncJumpWidth = 1;

    hfdcan1.Init.MessageRAMOffset = 0; 
    hfdcan1.Init.StdFiltersNbr = 0; 
    hfdcan1.Init.ExtFiltersNbr = 0;

    // FIFO 엘리먼트 크기를 64바이트로 확장 (멀티-레지스터 배칭 지원)
    hfdcan1.Init.RxFifo0ElmtsNbr = 16;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan1.Init.TxEventsNbr = 0; 
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 16; 
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; 
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) return false;
    
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, data_pre * (hfdcan1.Init.DataTimeSeg1 + 1), 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1); 
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return false;

    // 기본 Tx 헤더
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; 
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; 
    TxHeader.BitRateSwitch = FDCAN_BRS_ON; 
    TxHeader.FDFormat = FDCAN_FD_CAN; 
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 
    TxHeader.MessageMarker = 0;

    return true;
}

// ============================================================================
// [Core transact: 8바이트 CAN2.0 호환 모드]
// 설정용 (enable, disable, setMotionMode 등) - 느려도 OK
// ============================================================================
bool MD80Comm::transact(uint8_t* txBuf, uint8_t* rxBuf, uint32_t timeout_us) {
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
    }

    TxHeader.Identifier = motor_id_;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) return false;

    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                if (RxHeader.Identifier == motor_id_ && 
                    RxData[0] == txBuf[0] && 
                    RxData[2] == txBuf[2] && 
                    RxData[3] == txBuf[3]) {
                    memcpy(rxBuf, RxData, 8);
                    return true;
                }
            }
        }
    }
    return false;
}

// ============================================================================
// [transactFD - FDCAN 가변 길이 멀티-레지스터 통신]
// ============================================================================
bool MD80Comm::transactFD(uint8_t* txBuf, uint32_t txLen, uint8_t* rxBuf, uint32_t rxLen, uint32_t timeout_us) {
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
    }

    TxHeader.Identifier = motor_id_;
    
    if      (txLen <= 8)  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    else if (txLen <= 12) TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if (txLen <= 16) TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if (txLen <= 20) TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if (txLen <= 24) TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if (txLen <= 32) TxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if (txLen <= 48) TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else                  TxHeader.DataLength = FDCAN_DLC_BYTES_64;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) return false;

    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                if (RxHeader.Identifier == motor_id_ && RxData[0] == txBuf[0]) {
                    uint32_t copyLen = (rxLen <= 64) ? rxLen : 64;
                    memcpy(rxBuf, RxData, copyLen);
                    return true;
                }
            }
        }
    }
    return false;
}

// ────────────────────────────────────────────────
// Register access (설정 시에만 사용)
// ────────────────────────────────────────────────
bool MD80Comm::writeRegisterU8(uint16_t regID, uint8_t value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t)); 
    txBuf[4] = value;
    return transact(txBuf, rxBuf, 500);
}

bool MD80Comm::writeRegisterFloat(uint16_t regID, float value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t));
    memcpy(&txBuf[4], &value, sizeof(float));
    return transact(txBuf, rxBuf, 500);
}

bool MD80Comm::readRegisterFloat(uint16_t regID, float& value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t));
    if (!transact(txBuf, rxBuf, 500)) return false;
    memcpy(&value, &rxBuf[4], sizeof(float));
    return true;
}

// [진단용] uint32 레지스터 읽기 (firmwareVersion 등 float이 아닌 값 확인용)
bool MD80Comm::readRegisterU32(uint16_t regID, uint32_t& value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t));
    if (!transact(txBuf, rxBuf, 500)) return false;
    memcpy(&value, &rxBuf[4], sizeof(uint32_t));
    return true;
}

// ────────────────────────────────────────────────
// High-level commands (설정 시에만 사용, 느려도 OK)
// ────────────────────────────────────────────────
bool MD80Comm::enable() {
    return writeRegisterU8(MD80_REG_CONTROL_WORD, MD80_CONTROL_WORD_ENABLE);
}

bool MD80Comm::disable() {
    return writeRegisterU8(MD80_REG_CONTROL_WORD, MD80_CONTROL_WORD_DISABLE);
}

bool MD80Comm::zero() {
    return writeRegisterU8(MD80_REG_ZERO, 0x01);
}

bool MD80Comm::blink() {
    return writeRegisterU8(MD80_REG_BLINK, 0x01);
}

bool MD80Comm::clearErrors() {
    return writeRegisterU8(MD80_REG_CLEAR_ERRORS, 0x01);
}

bool MD80Comm::setMotionMode(MD80MotionMode mode) {
    return writeRegisterU8(MD80_REG_MOTION_MODE_COMMAND, static_cast<uint8_t>(mode));
}

bool MD80Comm::setTargetTorque(float torque_Nm) {
    return writeRegisterFloat(MD80_REG_TARGET_TORQUE, torque_Nm);
}

bool MD80Comm::setTargetPosition(float pos_rad) {
    return writeRegisterFloat(MD80_REG_TARGET_POS, pos_rad);
}

bool MD80Comm::setTargetVelocity(float vel_rads) {
    return writeRegisterFloat(MD80_REG_TARGET_VEL, vel_rads);
}

bool MD80Comm::setImpedanceParams(float kp, float kd) {
    // [33차] MD80 은 프레임 처리 전후 ~50 µs 동안 다음 프레임을 버림 (hw_kin_dyn 09-07 실측). 연속 transact
    //   사이에 간격을 두지 않으면 두 번째 쓰기(kd)가 유실됨 (09-14 readback: kp=100, kd=0.25 기본값 잔존).
    delayMicroseconds(300);
    if (!writeRegisterFloat(MD80_REG_IMPEDANCE_KP, kp)) return false;
    delayMicroseconds(300);
    if (!writeRegisterFloat(MD80_REG_IMPEDANCE_KD, kd)) return false;
    delayMicroseconds(300);
    return true;
}

bool MD80Comm::getPosition(float& pos_rad) {
    return readRegisterFloat(MD80_REG_MAIN_ENCODER_POS, pos_rad);
}

bool MD80Comm::getVelocity(float& vel_rads) {
    return readRegisterFloat(MD80_REG_MAIN_ENCODER_VEL, vel_rads);
}

bool MD80Comm::getTorque(float& torque_Nm) {
    return readRegisterFloat(MD80_REG_TORQUE, torque_Nm);
}

// ============================================================================
// [최적화 핵심] sendTorqueAndReadState - FDCAN 멀티-레지스터 배칭
//
// 전략: 2-step 방식 (Write 1회 + Read 1회)
//
// Step 1: Write Frame (8B) - 목표 토크 설정
//   [0x42][0x00][0x0152(TARGET_TORQUE)][torque_float(4B)]
//
// Step 2: Read Frame (20B) - 위치+속도+토크 동시 읽기
//   TX: [0x41][0x00][0x0063(POS)][0x00000000][0x0062(VEL)][0x00000000][0x0066(TRQ)][0x00000000]
//   RX: [0x41][0x00][0x0063(POS)][pos_float ][0x0062(VEL)][vel_float ][0x0066(TRQ)][trq_float ]
//
// 시간 예산 (5Mbps BRS):
//   - Step 1: 8B TX + 모터 처리 + 8B RX ≈ 100us
//   - Step 2: 20B TX + 모터 처리 + 20B RX ≈ 150us
//   - 합계: ~250us / 모터
//   - 4개 모터: 4 * 250us = 1000us → 1kHz 달성!
// ============================================================================
bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
    state.valid = false;

    // ── Step 1: 목표 토크 Write (8B, 기존 transact 사용) ──
    bool ok_torque = setTargetTorque(torque_Nm);
    if (!ok_torque) return false;

    // ── Step 2: 위치 + 속도 + 토크 동시 Read (20B FDCAN 멀티-레지스터) ──
    uint8_t txBuf[20] = {0};
    uint8_t rxBuf[20] = {0};

    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;  // 0x41
    txBuf[1] = 0x00;                          // Padding

    uint16_t reg_pos = MD80_REG_MAIN_ENCODER_POS;
    memcpy(&txBuf[2], &reg_pos, sizeof(uint16_t));

    uint16_t reg_vel = MD80_REG_MAIN_ENCODER_VEL;
    memcpy(&txBuf[8], &reg_vel, sizeof(uint16_t));

    uint16_t reg_trq = MD80_REG_TORQUE;
    memcpy(&txBuf[14], &reg_trq, sizeof(uint16_t));

    if (!transactFD(txBuf, 20, rxBuf, 20, 200)) {
        bool ok_pos = getPosition(state.position);
        if (ok_pos) {
            state.velocity = 0.0f;
            state.torque = torque_Nm;
            state.valid = true;
            return true;
        }
        return false;
    }

    memcpy(&state.position, &rxBuf[4],  sizeof(float));
    memcpy(&state.velocity, &rxBuf[10], sizeof(float));
    memcpy(&state.torque,   &rxBuf[16], sizeof(float));

    state.valid = true;
    return true;
}

// ============================================================================
// [1kHz 파이프라인] sendTorqueWriteAsync
// 0x42 WRITE_REGISTER_FD 프레임(targetTorque, 8B)을 TX FIFO에 큐잉만 하고 즉시 리턴.
// ============================================================================
bool MD80Comm::sendTorqueWriteAsync(uint32_t can_id, float torque_Nm) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD; // 0x42
    txBuf[1] = 0x00;                          // Padding
    uint16_t reg = MD80_REG_TARGET_TORQUE;    // 0x0152
    memcpy(&txBuf[2], &reg, sizeof(uint16_t));
    memcpy(&txBuf[4], &torque_Nm, sizeof(float));

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

bool MD80Comm::sendVelocityWriteAsync(uint32_t can_id, float vel_rads) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD; // 0x42
    txBuf[1] = 0x00;                          // Padding
    uint16_t reg = MD80_REG_TARGET_VEL;       // 0x0151
    memcpy(&txBuf[2], &reg, sizeof(uint16_t));
    memcpy(&txBuf[4], &vel_rads, sizeof(float));

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

bool MD80Comm::sendPositionWriteAsync(uint32_t can_id, float pos_rad) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD; // 0x42
    txBuf[1] = 0x00;                          // Padding
    uint16_t reg = MD80_REG_TARGET_POS;       // 0x0150
    memcpy(&txBuf[2], &reg, sizeof(uint16_t));
    memcpy(&txBuf[4], &pos_rad, sizeof(float));

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

// ============================================================================
// [v2 핵심] sendPosReadAsync - 8바이트 단일 레지스터 Position Read
// ----------------------------------------------------------------------------
// 기존 sendStateReadAsync는 20바이트 멀티-레지스터(pos+vel+torque) 프레임을 사용했음.
// 응답도 20바이트 → 4개 모터의 응답이 RX FIFO를 넘쳐 2-by-2 batch가 불가피했음.
//
// 이 함수는 Teensy multi_motor_control.ino와 동일하게 8바이트 단일 레지스터만 요청.
// 응답도 8바이트 → 4개 모터 응답 = 32바이트 → RX FIFO(16슬롯 × 64B)에 여유롭게 수용.
// ============================================================================
bool MD80Comm::sendStateReadAsync(uint32_t can_id) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;
    txBuf[1] = 0x00;

    uint16_t reg1 = MD80_REG_MAIN_ENCODER_POS;
    uint16_t reg2 = MD80_REG_MAIN_ENCODER_VEL;
    uint16_t reg3 = MD80_REG_TORQUE;

    memcpy(&txBuf[2], &reg1, sizeof(uint16_t));
    memcpy(&txBuf[4], &reg2, sizeof(uint16_t));
    memcpy(&txBuf[6], &reg3, sizeof(uint16_t));

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) {
        return false;
    }
    return true;
}

int MD80Comm::pollStateReads(const uint32_t* motor_ids, MD80State* states, bool* got,
                             int count, uint32_t timeout_us) {
    int received = 0;
    for (int i = 0; i < count; i++) got[i] = false;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];

    uint32_t start = micros();
    while (received < count && (uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                if (RxData[0] == MD80_FRAME_READ_REGISTER_FD) {
                    for (int i = 0; i < count; i++) {
                        if (RxHeader.Identifier == motor_ids[i] && !got[i]) {
                            // Parse 20-byte response dynamically in case the motor sorts the register IDs
                            uint32_t rxLen = 8;
                            switch (RxHeader.DataLength) {
                                case FDCAN_DLC_BYTES_8: rxLen = 8; break;
                                case FDCAN_DLC_BYTES_12: rxLen = 12; break;
                                case FDCAN_DLC_BYTES_16: rxLen = 16; break;
                                case FDCAN_DLC_BYTES_20: rxLen = 20; break;
                                case FDCAN_DLC_BYTES_24: rxLen = 24; break;
                                case FDCAN_DLC_BYTES_32: rxLen = 32; break;
                                case FDCAN_DLC_BYTES_48: rxLen = 48; break;
                                case FDCAN_DLC_BYTES_64: rxLen = 64; break;
                            }
                            
                            int idx = 2;
                            while (idx + 5 < rxLen) { // 2 bytes for regID, 4 bytes for float value
                                uint16_t reg;
                                memcpy(&reg, &RxData[idx], 2);
                                float val;
                                memcpy(&val, &RxData[idx + 2], 4);
                                
                                if (reg == MD80_REG_MAIN_ENCODER_POS) states[i].position = val;
                                else if (reg == MD80_REG_MAIN_ENCODER_VEL) states[i].velocity = val;
                                else if (reg == MD80_REG_TORQUE) states[i].torque = val;
                                
                                idx += 6;
                            }
                            got[i] = true;
                            received++;
                            break;
                        }
                    }
                }
            }
        }
    }
    return received;
}

bool MD80Comm::sendPosReadAsync(uint32_t can_id) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD; // 0x41
    txBuf[1] = 0x00;                         // Padding
    uint16_t reg = MD80_REG_MAIN_ENCODER_POS; // 0x0063
    memcpy(&txBuf[2], &reg, sizeof(uint16_t));
    // txBuf[4..7] = 0 (read 요청이므로 값 불필요)

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

// ============================================================================
// [v2 핵심] pollPosReads - 8바이트 Position 응답 수거
// ----------------------------------------------------------------------------
// RX FIFO를 드레인하며 0x41(Read 응답) 프레임만 골라 CAN ID로 매칭.
// Write ack(0x42)는 자동으로 무시.
// ============================================================================
int MD80Comm::pollPosReads(const uint32_t* motor_ids, float* positions, bool* got,
                            int count, uint32_t timeout_us) {
    for (int i = 0; i < count; i++) {
        got[i] = false;
    }

    int received = 0;
    uint32_t start = micros();

    while (received < count && (uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                if (RxData[0] == MD80_FRAME_READ_REGISTER_FD) {
                    uint16_t reg_id;
                    memcpy(&reg_id, &RxData[2], sizeof(uint16_t));
                    if (reg_id == MD80_REG_MAIN_ENCODER_POS) {
                        for (int i = 0; i < count; i++) {
                            if (!got[i] && motor_ids[i] == RxHeader.Identifier) {
                                memcpy(&positions[i], &RxData[4], sizeof(float));
                                got[i] = true;
                                received++;
                                break;
                            }
                        }
                    }
                }
                // Write ack(0x42) 등은 무시하고 계속 드레인
            }
        }
    }
    return received;
}

// ============================================================================
// [진단용] getBusDiagnostics
// ============================================================================
// Stable 8-byte single-register torque request/response.
bool MD80Comm::sendTorqueReadAsync(uint32_t can_id) {
    uint8_t txBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD; // 0x41
    txBuf[1] = 0x00;
    uint16_t reg = MD80_REG_TORQUE;         // 0x0064
    memcpy(&txBuf[2], &reg, sizeof(uint16_t));

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

int MD80Comm::pollTorqueReads(const uint32_t* motor_ids, float* torques, bool* got,
                              int count, uint32_t timeout_us) {
    for (int i = 0; i < count; i++) got[i] = false;

    int received = 0;
    uint32_t start = micros();
    while (received < count && (uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0 &&
            HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            if (RxData[0] == MD80_FRAME_READ_REGISTER_FD) {
                uint16_t reg_id;
                memcpy(&reg_id, &RxData[2], sizeof(uint16_t));
                if (reg_id == MD80_REG_TORQUE) {
                    for (int i = 0; i < count; i++) {
                        if (!got[i] && motor_ids[i] == RxHeader.Identifier) {
                            memcpy(&torques[i], &RxData[4], sizeof(float));
                            got[i] = true;
                            received++;
                            break;
                        }
                    }
                }
            }
        }
    }
    return received;
}

// FDCAN bus diagnostics.
FDCANDiag MD80Comm::getBusDiagnostics() {
    FDCANDiag d = {false, false, false, 0, 0};

    FDCAN_ProtocolStatusTypeDef ps = {0};
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    d.busOff = (ps.BusOff != 0);
    d.errorPassive = (ps.ErrorPassive != 0);
    d.errorWarning = (ps.Warning != 0);

    FDCAN_ErrorCountersTypeDef ec = {0};
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
    d.txErrorCount = (uint8_t)ec.TxErrorCnt;
    d.rxErrorCount = (uint8_t)ec.RxErrorCnt;

    return d;
}

void MD80Comm::flushRxFifo() {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
    }
}

// [v4 추가] Position과 Torque 2개의 레지스터 동시 요청
bool MD80Comm::sendPosAndTorqueReadAsync(uint32_t can_id) {
    uint8_t txBuf[16] = {0}; // 16바이트 할당 (FDCAN_DLC_BYTES_16)
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;
    txBuf[1] = 0x00;

    uint16_t reg1 = MD80_REG_MAIN_ENCODER_POS;
    uint16_t reg2 = MD80_REG_TORQUE;

    // 첫 번째 레지스터 (POS)
    memcpy(&txBuf[2], &reg1, sizeof(uint16_t));
    // txBuf[4] ~ txBuf[7]은 Float 크기(4바이트)만큼 0x00 패딩으로 비워둠 (모터가 응답 시 값을 채워넣음)

    // 두 번째 레지스터 (TORQUE)
    memcpy(&txBuf[8], &reg2, sizeof(uint16_t));
    // txBuf[10] ~ txBuf[13]은 Float 크기(4바이트)만큼 0x00 패딩으로 비워둠

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) == HAL_OK;
}

int MD80Comm::pollPosAndTorqueReads(const uint32_t* motor_ids, float* positions, float* torques, bool* got,
                                    int count, uint32_t timeout_us) {
    int received = 0;
    for (int i = 0; i < count; i++) got[i] = false;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];

    uint32_t start = micros();
    while (received < count && (uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                if (RxData[0] == MD80_FRAME_READ_REGISTER_FD) {
                    for (int i = 0; i < count; i++) {
                        if (RxHeader.Identifier == motor_ids[i] && !got[i]) {
                            uint32_t rxLen = 8;
                            switch (RxHeader.DataLength) {
                                case FDCAN_DLC_BYTES_8: rxLen = 8; break;
                                case FDCAN_DLC_BYTES_12: rxLen = 12; break;
                                case FDCAN_DLC_BYTES_16: rxLen = 16; break;
                                case FDCAN_DLC_BYTES_20: rxLen = 20; break;
                                case FDCAN_DLC_BYTES_24: rxLen = 24; break;
                                case FDCAN_DLC_BYTES_32: rxLen = 32; break;
                                case FDCAN_DLC_BYTES_48: rxLen = 48; break;
                                case FDCAN_DLC_BYTES_64: rxLen = 64; break;
                            }
                            
                            int idx = 2;
                            while (idx + 5 < rxLen) { // 2 bytes for regID, 4 bytes for float value
                                uint16_t reg;
                                memcpy(&reg, &RxData[idx], 2);
                                float val;
                                memcpy(&val, &RxData[idx + 2], 4);
                                
                                if (reg == MD80_REG_MAIN_ENCODER_POS) positions[i] = val;
                                else if (reg == MD80_REG_TORQUE) torques[i] = val;
                                
                                idx += 6;
                            }
                            got[i] = true;
                            received++;
                            break;
                        }
                    }
                }
            }
        }
    }
    return received;
}
