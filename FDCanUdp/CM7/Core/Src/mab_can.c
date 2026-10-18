#include "mab_can.h"

errorMab_E writeRegisterPrepareDataFrame(void*            buffer,
                                         const size_t     bufferSize,
                                         const uint16_t   registerId,
                                         const void*      data,
                                         const dataType_E type,
                                         const bool       defaultResponse)
{
    if (buffer == NULL || data == NULL)
        return ERROR_NULL_PTR;

    memset(buffer, 0, bufferSize);

    uint8_t* buf = (uint8_t*)buffer;

    if (defaultResponse)
        buf[0] = FRAME_WRITE_REGISTER_DEFAULT_RESPONSE;
    else
        buf[0] = FRAME_WRITE_REGISTER;
    buf[1] = 0x00;

    memcpy(buf + 2, &registerId, sizeof(uint16_t));

    switch (type)
    {
        case INT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int8_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (int8_t*)data, sizeof(int8_t));
            break;
        case INT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int16_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (int16_t*)data, sizeof(int16_t));
            break;
        case INT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int32_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (int32_t*)data, sizeof(int32_t));
            break;
        case INT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int64_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (int64_t*)data, sizeof(int64_t));
            break;
        case UINT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint8_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (uint8_t*)data, sizeof(uint8_t));
            break;
        case UINT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint16_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (uint16_t*)data, sizeof(uint16_t));
            break;
        case UINT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint32_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (uint32_t*)data, sizeof(uint32_t));
            break;
        case UINT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint64_t))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (uint64_t*)data, sizeof(uint64_t));
            break;
        case FLOAT:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(float))
                return ERROR_BUF_SIZE;
            memcpy(buf + 2 + sizeof(uint16_t), (float*)data, sizeof(float));
            break;
        default:
            return ERROR_INVALID_TYPE;
    }

    return OK;
}

errorMab_E readRegisterPrepareDataFrame(void*            buffer,
                                        const size_t     bufferSize,
                                        const uint16_t   registerId,
                                        const dataType_E type)
{
    if (buffer == NULL)
        return ERROR_NULL_PTR;

    memset(buffer, 0, bufferSize);

    uint8_t* buf = (uint8_t*)buffer;

    buf[0] = FRAME_READ_REGISTER;
    buf[1] = 0x00;

    memcpy(buf + 2, &registerId, sizeof(uint16_t));

    switch (type)
    {
        case INT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int8_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (int8_t)0;
            break;
        case INT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int16_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (int16_t)0;
            break;
        case INT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int32_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (int32_t)0;
            break;
        case INT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int64_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (int64_t)0;
            break;
        case UINT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint8_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (uint8_t)0;
            break;
        case UINT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint16_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (uint16_t)0;
            break;
        case UINT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint32_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (uint32_t)0;
            break;
        case UINT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint64_t))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (uint64_t)0;
            break;
        case FLOAT:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(float))
                return ERROR_BUF_SIZE;
            buf[2 + sizeof(uint16_t)] = (float)0;
            break;
        default:
            return ERROR_INVALID_TYPE;
    }

    return OK;
}

errorMab_E parseResponse(const void*      buffer,
                         const size_t     bufferSize,
                         const uint8_t    frameId,
                         const uint16_t   registerId,
                         void*            data,
                         const dataType_E type)
{
    if (buffer == NULL || data == NULL)
        return ERROR_NULL_PTR;

    uint8_t* buf = (uint8_t*)buffer;

    if ((uint8_t)buf[0] != frameId)
        return ERROR_INVALID_FRAME;

    if (((uint16_t)buf[2] | ((uint16_t)buf[3] << 8)) != registerId)
        return ERROR_INVALID_REG;

    switch (type)
    {
        case INT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int8_t))
                return ERROR_BUF_SIZE;
            memcpy((int8_t*)data, buf + 2 + sizeof(uint16_t), sizeof(int8_t));
            break;
        case INT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int16_t))
                return ERROR_BUF_SIZE;
            memcpy((int16_t*)data, buf + 2 + sizeof(uint16_t), sizeof(int16_t));
            break;
        case INT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int32_t))
                return ERROR_BUF_SIZE;
            memcpy((int32_t*)data, buf + 2 + sizeof(uint16_t), sizeof(int32_t));
            break;
        case INT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(int64_t))
                return ERROR_BUF_SIZE;
            memcpy((int64_t*)data, buf + 2 + sizeof(uint16_t), sizeof(int64_t));
            break;
        case UINT8:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint8_t))
                return ERROR_BUF_SIZE;
            memcpy((uint8_t*)data, buf + 2 + sizeof(uint16_t), sizeof(uint8_t));
            break;
        case UINT16:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint16_t))
                return ERROR_BUF_SIZE;
            memcpy((uint16_t*)data, buf + 2 + sizeof(uint16_t), sizeof(uint16_t));
            break;
        case UINT32:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint32_t))
                return ERROR_BUF_SIZE;
            memcpy((uint32_t*)data, buf + 2 + sizeof(uint16_t), sizeof(uint32_t));
            break;
        case UINT64:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(uint64_t))
                return ERROR_BUF_SIZE;
            memcpy((uint64_t*)data, buf + 2 + sizeof(uint16_t), sizeof(uint64_t));
            break;
        case FLOAT:
            if (bufferSize < 2 + sizeof(uint16_t) + sizeof(float))
                return ERROR_BUF_SIZE;
            memcpy((float*)data, buf + 2 + sizeof(uint16_t), sizeof(float));
            break;
        default:
            return ERROR_INVALID_TYPE;
    }

    return OK;
}

errorMab_E setMotionModePrepareDataFrame(void*                 buffer,
                                         const size_t          bufferSize,
                                         const motionModeMab_E mode)
{
    uint8_t modeVar = (uint8_t)mode;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_MOTION_MODE_COMMAND, &modeVar, UINT8, false);
}

errorMab_E setMotionModeParseResponse(const void*           buffer,
                                      const size_t          bufferSize,
                                      const motionModeMab_E mode)
{
    uint8_t    data = 0;
    errorMab_E err  = parseResponse(
        buffer, bufferSize, FRAME_WRITE_REGISTER, REG_MOTION_MODE_COMMAND, &data, UINT8);

    if (err)
    {
        if (data == (uint8_t)mode)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E getMotionModePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    return readRegisterPrepareDataFrame(buffer, bufferSize, REG_MOTION_MODE_STATUS, UINT8);
}

errorMab_E getMotionModeParseResponse(const void* buffer, const size_t bufferSize, uint8_t* data)
{
    return parseResponse(
        buffer, bufferSize, FRAME_READ_REGISTER, REG_MOTION_MODE_STATUS, data, UINT8);
}

errorMab_E getQuickStatusPrepareDataFrame(void* buffer, const size_t bufferSize)
{
    return readRegisterPrepareDataFrame(buffer, bufferSize, REG_QUICK_STATUS, UINT16);
}

errorMab_E getQuickStatusParseResponse(const void* buffer, const size_t bufferSize, uint16_t* data)
{
    return parseResponse(buffer, bufferSize, FRAME_READ_REGISTER, REG_QUICK_STATUS, data, UINT16);
}

errorMab_E getVelocityPrepareDataFrame(void* buffer, const size_t bufferSize)
{
    const bool useMainEncoder = true;
    if (useMainEncoder)
        return readRegisterPrepareDataFrame(buffer, bufferSize, REG_MAIN_ENCODER_VEL, FLOAT);
    else
        return readRegisterPrepareDataFrame(buffer, bufferSize, REG_OUTPUT_ENCODER_VEL, FLOAT);
}

errorMab_E getVelocityParseResponse(const void* buffer, const size_t bufferSize, float* data)
{
    const bool useMainEncoder = true;
    if (useMainEncoder)
        return parseResponse(
            buffer, bufferSize, FRAME_READ_REGISTER, REG_MAIN_ENCODER_VEL, data, FLOAT);
    else
        return parseResponse(
            buffer, bufferSize, FRAME_READ_REGISTER, REG_OUTPUT_ENCODER_VEL, data, FLOAT);
}

errorMab_E getPositionPrepareDataFrame(void* buffer, const size_t bufferSize)
{
    const bool useMainEncoder = true;
    if (useMainEncoder)
        return readRegisterPrepareDataFrame(buffer, bufferSize, REG_MAIN_ENCODER_POS, FLOAT);
    else
        return readRegisterPrepareDataFrame(buffer, bufferSize, REG_OUTPUT_ENCODER_POS, FLOAT);
}

errorMab_E getPositionParseResponse(const void* buffer, const size_t bufferSize, float* data)
{
    const bool useMainEncoder = true;
    if (useMainEncoder)
        return parseResponse(
            buffer, bufferSize, FRAME_READ_REGISTER, REG_MAIN_ENCODER_POS, data, FLOAT);
    else
        return parseResponse(
            buffer, bufferSize, FRAME_READ_REGISTER, REG_OUTPUT_ENCODER_POS, data, FLOAT);
}

errorMab_E getTorquePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    return readRegisterPrepareDataFrame(buffer, bufferSize, REG_MOTOR_TORQUE, FLOAT);
}

errorMab_E getTorqueParseResponse(const void* buffer, const size_t bufferSize, float* data)
{
    return parseResponse(buffer, bufferSize, FRAME_READ_REGISTER, REG_MOTOR_TORQUE, data, FLOAT);
}

errorMab_E setVelocityPrepareDataFrame(void* buffer, const size_t bufferSize, const float velocity)
{
    float velocityVar = (float)velocity;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_TARGET_VEL, &velocityVar, FLOAT, false);
}

errorMab_E setVelocityParseResponse(const void*  buffer,
                                    const size_t bufferSize,
                                    const float  velocity)
{
    float      data = 0.f;
    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_TARGET_VEL, &data, FLOAT);

    if (err)
    {
        if (data == velocity)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E setProfileVelocityPrepareDataFrame(void*        buffer,
                                              const size_t bufferSize,
                                              const float  velocity)
{
    float velocityVar = (float)velocity;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_PROFILE_VEL, &velocityVar, FLOAT, false);
}

errorMab_E setProfileVelocityParseResponse(const void*  buffer,
                                           const size_t bufferSize,
                                           const float  velocity)
{
    float      data = 0.f;
    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_PROFILE_VEL, &data, FLOAT);

    if (err)
    {
        if (data == velocity)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E setProfileAccelerationPrepareDataFrame(void*        buffer,
                                                  const size_t bufferSize,
                                                  const float  acceleration)
{
    float accelerationVar = (float)acceleration;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_PROFILE_ACC, &accelerationVar, FLOAT, false);
}

errorMab_E setProfileAccelerationParseResponse(const void*  buffer,
                                               const size_t bufferSize,
                                               const float  acceleration)
{
    float      data = 0.f;
    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_PROFILE_ACC, &data, FLOAT);

    if (err)
    {
        if (data == acceleration)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E setProfileDecelerationPrepareDataFrame(void*        buffer,
                                                  const size_t bufferSize,
                                                  const float  deceleration)
{
    float decelerationVar = (float)deceleration;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_PROFILE_DEC, &decelerationVar, FLOAT, false);
}

errorMab_E setProfileDecelerationParseResponse(const void*  buffer,
                                               const size_t bufferSize,
                                               const float  deceleration)
{
    float      data = 0.f;
    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_PROFILE_DEC, &data, FLOAT);

    if (err)
    {
        if (data == deceleration)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E getDCVoltagePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    return readRegisterPrepareDataFrame(buffer, bufferSize, REG_DC_VOLTAGE, FLOAT);
}

errorMab_E getDCVoltageParseResponse(const void* buffer, const size_t bufferSize, float* data)
{
    return parseResponse(buffer, bufferSize, FRAME_READ_REGISTER, REG_DC_VOLTAGE, data, FLOAT);
}

errorMab_E setStateShutdownPrepareDataFrame(void* buffer, const size_t bufferSize)
{
    uint16_t controlWord = CONTROL_WORD_SHUTDOWN;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_CONTROL_WORD, &controlWord, UINT16, false);
}

errorMab_E setStateShutdownParseResponse(const void* buffer, const size_t bufferSize)
{
    const uint16_t controlWord = CONTROL_WORD_SHUTDOWN;
    uint16_t       data        = 0;
    errorMab_E     err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_CONTROL_WORD, &data, UINT16);

    if (err)
    {
        if (data == controlWord)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E setStateEnablePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    uint16_t controlWord = CONTROL_WORD_ENABLE;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_CONTROL_WORD, &controlWord, UINT16, false);
}

errorMab_E setStateEnableParseResponse(const void* buffer, const size_t bufferSize)
{
    const uint16_t controlWord = CONTROL_WORD_ENABLE;
    uint16_t       data        = 0;

    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_CONTROL_WORD, &data, UINT16);

    if (err)
    {
        if (data == controlWord)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E setStateDisablePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    uint16_t controlWord = CONTROL_WORD_DISABLE;
    return writeRegisterPrepareDataFrame(
        buffer, bufferSize, REG_CONTROL_WORD, &controlWord, UINT16, false);
}

errorMab_E setStateDisableParseResponse(const void* buffer, const size_t bufferSize)
{
    const uint16_t controlWord = CONTROL_WORD_DISABLE;
    uint16_t       data        = 0;

    errorMab_E err =
        parseResponse(buffer, bufferSize, FRAME_WRITE_REGISTER, REG_CONTROL_WORD, &data, UINT16);

    if (err)
    {
        if (data == controlWord)
            return OK;
        else
            return ERROR_OPERATION_FAILED;
    }
    else
        return err;
}

errorMab_E pingPrepareDataFrame(void* buffer, const size_t bufferSize)
{
    return getQuickStatusPrepareDataFrame(buffer, bufferSize);
}

errorMab_E pingParseResponse(const void* buffer, const size_t bufferSize)
{
    if (buffer == NULL)
        return ERROR_NULL_PTR;
    if (bufferSize < PING_FRAME_SIZE)
        return ERROR_BUF_SIZE;

    const uint8_t* buf = (const uint8_t*)buffer;
    if (buf[0] != FRAME_READ_REGISTER)
        return ERROR_INVALID_FRAME;

    return OK;
}

errorMab_E scanDrivers(fdcanDataBitrate_E dataBitrate,
                       pingTxRxCb_F       txRxCb,
                       uint32_t*          foundIds,
                       size_t             maxIds,
                       size_t*            foundCount)
{
    if (txRxCb == NULL || foundIds == NULL || foundCount == NULL)
        return ERROR_NULL_PTR;
    if (maxIds == 0)
        return ERROR_BUF_SIZE;
    if (dataBitrate != FDCAN_DATA_BITRATE_1M && dataBitrate != FDCAN_DATA_BITRATE_2M &&
        dataBitrate != FDCAN_DATA_BITRATE_5M && dataBitrate != FDCAN_DATA_BITRATE_8M)
        return ERROR_INVALID_TYPE;

    *foundCount = 0;

    uint8_t txBuf[PING_FRAME_SIZE];
    uint8_t rxBuf[PING_FRAME_SIZE];

    for (uint32_t id = CAN_MIN_ID; id <= CAN_MAX_ID && *foundCount < maxIds; id++)
    {
        if (pingPrepareDataFrame(txBuf, sizeof(txBuf)) != OK)
            continue;
        if (txRxCb(id, txBuf, sizeof(txBuf), rxBuf, sizeof(rxBuf)) != OK)
            continue;
        if (pingParseResponse(rxBuf, sizeof(rxBuf)) != OK)
            continue;

        foundIds[(*foundCount)++] = id;
    }

    return OK;
}

/* Frame layout (20 bytes):
 * [0]    frame ID (0x41)
 * [1]    padding (0x00)
 * [2-3]  REG_MAIN_ENCODER_POS
 * [4-7]  float value (position)
 * [8-9]  REG_MAIN_ENCODER_VEL
 * [10-13] float value (velocity)
 * [14-15] REG_MOTOR_TORQUE
 * [16-19] float value (torque)
 */
#define MOTOR_STATE_FRAME_SIZE 20U

errorMab_E getMotorStatePrepareDataFrame(void* buffer, const size_t bufferSize)
{
    if (buffer == NULL)
        return ERROR_NULL_PTR;
    if (bufferSize < MOTOR_STATE_FRAME_SIZE)
        return ERROR_BUF_SIZE;

    memset(buffer, 0, bufferSize);
    uint8_t* buf = (uint8_t*)buffer;

    buf[0] = FRAME_READ_REGISTER_FDCAN;
    buf[1] = 0x00;

    uint16_t regPos = REG_MAIN_ENCODER_POS;
    uint16_t regVel = REG_MAIN_ENCODER_VEL;
    uint16_t regTor = REG_MOTOR_TORQUE;

    memcpy(buf + 2,  &regPos, sizeof(uint16_t));
    memcpy(buf + 8,  &regVel, sizeof(uint16_t));
    memcpy(buf + 14, &regTor, sizeof(uint16_t));

    return OK;
}

errorMab_E getMotorStateParseResponse(const void*   buffer,
                                      const size_t  bufferSize,
                                      motorState_T* state)
{
    if (buffer == NULL || state == NULL)
        return ERROR_NULL_PTR;
    if (bufferSize < MOTOR_STATE_FRAME_SIZE)
        return ERROR_BUF_SIZE;

    const uint8_t* buf = (const uint8_t*)buffer;

    if (buf[0] != FRAME_READ_REGISTER_FDCAN)
        return ERROR_INVALID_FRAME;

    uint16_t regId;

    memcpy(&regId, buf + 2, sizeof(uint16_t));
    if (regId != REG_MAIN_ENCODER_POS)
        return ERROR_INVALID_REG;

    memcpy(&regId, buf + 8, sizeof(uint16_t));
    if (regId != REG_MAIN_ENCODER_VEL)
        return ERROR_INVALID_REG;

    memcpy(&regId, buf + 14, sizeof(uint16_t));
    if (regId != REG_MOTOR_TORQUE)
        return ERROR_INVALID_REG;

    memcpy(&state->mainEncoderPosition, buf + 4,  sizeof(float));
    memcpy(&state->mainEncoderVelocity, buf + 10, sizeof(float));
    memcpy(&state->motorTorque,         buf + 16, sizeof(float));

    return OK;
}
