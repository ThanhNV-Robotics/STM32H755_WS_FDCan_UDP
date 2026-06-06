#ifndef MAB_CAN_H
#define MAB_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief CAN frame type for writing a MD device register and getting default response. */
#define FRAME_WRITE_REGISTER_DEFAULT_RESPONSE 0x40
/** @brief CAN frame type for reading from a MD device register (FDCAN, up to 64 bytes). */
#define FRAME_READ_REGISTER_FDCAN 0x41
/** @brief CAN frame type for reading from a MD device register (CAN2.0, max 8 bytes). */
#define FRAME_READ_REGISTER 0x43
/** @brief CAN frame type for writing to a MD device register. */
#define FRAME_WRITE_REGISTER 0x44
/** @brief Default response frame ID for MAB CAN protocol. */
#define RESPONSE_DEFAULT 0xA0

/** @brief Minimum valid CAN device ID. */
#define CAN_MIN_ID ((uint32_t)10)
/** @brief Maximum valid CAN device ID. */
#define CAN_MAX_ID ((uint32_t)2000)

/** @brief Size of the ping request/response frame in bytes. */
#define PING_FRAME_SIZE ((size_t)6)

/** @brief Output encoder velocity register ID. */
#define REG_OUTPUT_ENCODER_VEL 0x023
/** @brief Output encoder position register ID. */
#define REG_OUTPUT_ENCODER_POS 0x024

/** @brief Main encoder velocity register ID. */
#define REG_MAIN_ENCODER_VEL 0x062
/** @brief Main encoder position register ID. */
#define REG_MAIN_ENCODER_POS 0x063
/** @brief Motor torque measurement register ID. */
#define REG_MOTOR_TORQUE 0x064

/** @brief Velocity setpoint for motion profile. */
#define REG_PROFILE_VEL 0x120
/** @brief Acceleration setpoint for motion profile. */
#define REG_PROFILE_ACC 0x121
/** @brief Deceleration setpoint for motion profile. */
#define REG_PROFILE_DEC 0x122

/** @brief Motion mode command register ID. */
#define REG_MOTION_MODE_COMMAND 0x140
/** @brief Motion mode status register ID. */
#define REG_MOTION_MODE_STATUS 0x141
/** @brief Register for setting the state of the device. */
#define REG_CONTROL_WORD 0x142

/** @brief Target velocity register ID. */
#define REG_TARGET_VEL 0x151

/** @brief Quick status register (used for high-level health/status info). */
#define REG_QUICK_STATUS 0x805
/** @brief DC bus voltage monitoring register ID. */
#define REG_DC_VOLTAGE 0x811

/** @brief Control word for enabling the device. */
#define CONTROL_WORD_ENABLE 0x27
/** @brief Control word for disabling the device. */
#define CONTROL_WORD_DISABLE 0x40
/** @brief Control word for shutting down the device. */
#define CONTROL_WORD_SHUTDOWN 0x06

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

	/**
	 * @brief Enumeration of supported data types for register operations.
	 *
	 * This enum defines the possible data types that can be read from or written to device
	 * registers over the CAN interface. The type is used to correctly serialize or deserialize the
	 * binary data during communication.
	 */
	typedef enum
	{
		/** 8-bit signed integer */
		INT8,
		/** 16-bit signed integer */
		INT16,
		/** 32-bit signed integer */
		INT32,
		/** 64-bit signed integer */
		INT64,
		/** 8-bit unsigned integer */
		UINT8,
		/** 16-bit unsigned integer */
		UINT16,
		/** 32-bit unsigned integer */
		UINT32,
		/** 64-bit unsigned integer */
		UINT64,
		/** 32-bit IEEE-754 floating-point number */
		FLOAT
	} dataType_E;

	/**
	 * @brief Enumeration of possible error codes returned by MAB CAN functions.
	 *
	 * Functions in this library return values of this enum to indicate the result of an operation.
	 * `OK` indicates success, while other values indicate the type of error encountered.
	 */
	typedef enum
	{
		/** Unknown or unspecified error occurred */
		UNKNOWN_ERROR,
		/** Operation completed successfully */
		OK,
		/** Data type provided is not supported or invalid */
		ERROR_INVALID_TYPE,
		/** A required pointer argument was NULL */
		ERROR_NULL_PTR,
		/** Provided buffer is too small to hold data */
		ERROR_BUF_SIZE,
		/** Frame format or content is invalid */
		ERROR_INVALID_FRAME,
		/** Specified register ID is invalid or unsupported */
		ERROR_INVALID_REG,
		/** Operation failed during execution (e.g. mismatch, timeout) */
		ERROR_OPERATION_FAILED
	} errorMab_E;

	/**
	 * @brief Enumeration of supported motion control modes.
	 *
	 * These modes define how the motor controller interprets and responds to control commands, such
	 * as position, velocity, torque, or impedance targets.
	 */
	typedef enum
	{
		/** Idle mode, no control output */
		IDLE = 0,
		/** Position PID control */
		POSITION_PID = 1,
		/** Velocity PID control */
		VELOCITY_PID = 2,
		/** Direct raw torque control */
		RAW_TORQUE = 3,
		/** Impedance control, similar to spring-damper system */
		IMPEDANCE = 4,
		/** Position PID control with a trapezoidal profile (constant acceleration) */
		POSITION_PROFILE = 7,
		/** Velocity PID control with a trapezoidal profile (constant acceleration) */
		VELOCITY_PROFILE = 8
	} motionModeMab_E;

	/**
	 * @brief Prepares a data buffer for a MAB CAN write register frame.
	 *
	 * This function serializes the given register ID and data into the provided buffer so that it
	 * can be transmitted over the CAN bus to write a value to a device register.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @param registerId  ID of the register to write to.
	 * @param data        Pointer to the data to write into the register.
	 * @param type        Data type of the value to be written (e.g., UINT8, FLOAT).
	 * @param useDefaultResponse If true, the function will use the
	 * FRAME_WRITE_REGISTER_DEFAULT_RESPONSE type for writing a MD device register and getting
	 * default response.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E writeRegisterPrepareDataFrame(void*			  buffer,
											 const size_t	  bufferSize,
											 const uint16_t	  registerId,
											 const void*	  data,
											 const dataType_E type,
											 bool			  useDefaultResponse);

	/**
	 * @brief Prepares a data buffer for a MAB CAN read register frame.
	 *
	 * This function serializes the given register ID into the provided buffer so that it can be
	 * transmitted over the CAN bus to request a register read.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @param registerId  ID of the register to read from.
	 * @param type        Expected data type of the register to read.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E readRegisterPrepareDataFrame(void*			 buffer,
											const size_t	 bufferSize,
											const uint16_t	 registerId,
											const dataType_E type);

	/**
	 * @brief Parses a received MAB CAN response frame.
	 *
	 * This function validates and extracts data from a received CAN frame, checking that the frame
	 * ID and register ID match the expected values, and copies the register value into the provided
	 * output buffer.
	 *
	 * @param buffer      Pointer to the received CAN frame data.
	 * @param bufferSize  Size of the received buffer in bytes.
	 * @param frameId     Expected frame ID to validate.
	 * @param registerId  Expected register ID to validate.
	 * @param data        Pointer to memory where the parsed register value will be stored.
	 * @param type        Expected data type of the value in the response.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E parseResponse(const void*	  buffer,
							 const size_t	  bufferSize,
							 const uint8_t	  frameId,
							 const uint16_t	  registerId,
							 void*			  data,
							 const dataType_E type);

	/**
	 * @brief Prepares a data buffer to set the motion mode of the device.
	 *
	 * This function creates a CAN frame to command the device to change its current motion mode.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @param mode        Motion mode to be set.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setMotionModePrepareDataFrame(void*				   buffer,
											 const size_t		   bufferSize,
											 const motionModeMab_E mode);

	/**
	 * @brief Parses a response frame from a motion mode set command.
	 *
	 * This function validates the response from the device after sending a set motion mode command.
	 *
	 * @param buffer      Pointer to the received frame data.
	 * @param bufferSize  Size of the received buffer in bytes.
	 * @param mode        Motion mode that was expected to be acknowledged.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setMotionModeParseResponse(const void*			buffer,
										  const size_t			bufferSize,
										  const motionModeMab_E mode);

	/**
	 * @brief Prepares a frame to request the current motion mode.
	 *
	 * @param buffer      Pointer to the buffer to store the request frame.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getMotionModePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the response of a motion mode read command.
	 *
	 * @param buffer      Pointer to the received data buffer.
	 * @param bufferSize  Size of the buffer.
	 * @param data        Pointer to store the current motion mode.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getMotionModeParseResponse(const void*  buffer,
										  const size_t bufferSize,
										  uint8_t*	   data);

	/**
	 * @brief Prepares a frame to request quick status from the device.
	 *
	 * @param buffer      Pointer to the buffer to store the request frame.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getQuickStatusPrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the quick status response from the device.
	 *
	 * @param buffer      Pointer to the received data.
	 * @param bufferSize  Size of the received buffer.
	 * @param data        Pointer to store the status (16-bit value).
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getQuickStatusParseResponse(const void*	buffer,
										   const size_t bufferSize,
										   uint16_t*	data);

	/**
	 * @brief Prepares a frame to request velocity from the encoder.
	 *
	 * @param buffer         Pointer to the buffer to store the request.
	 * @param bufferSize     Size of the buffer.
	 * @return               Error code indicating success or failure.
	 */
	errorMab_E getVelocityPrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the velocity response from the encoder.
	 *
	 * @param buffer         Pointer to the received data buffer.
	 * @param bufferSize     Size of the buffer.
	 * @param data           Pointer to store the velocity.
	 * @return               Error code indicating success or failure.
	 */
	errorMab_E getVelocityParseResponse(const void* buffer, const size_t bufferSize, float* data);

	/**
	 * @brief Prepares a frame to request position from the encoder.
	 *
	 * @param buffer         Pointer to the buffer to store the request.
	 * @param bufferSize     Size of the buffer.
	 * @return               Error code indicating success or failure.
	 */
	errorMab_E getPositionPrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the position response from the encoder.
	 *
	 * @param buffer         Pointer to the received buffer.
	 * @param bufferSize     Size of the buffer.
	 * @param data           Pointer to store the position.
	 * @return               Error code indicating success or failure.
	 */
	errorMab_E getPositionParseResponse(const void* buffer, const size_t bufferSize, float* data);

	/**
	 * @brief Prepares a frame to request the motor torque value.
	 *
	 * @param buffer      Pointer to the buffer to store the request.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getTorquePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the torque response from the motor.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @param data        Pointer to store the torque value.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getTorqueParseResponse(const void* buffer, const size_t bufferSize, float* data);

	/**
	 * @brief Prepares a frame to set target velocity.
	 *
	 * @param buffer      Pointer to the buffer to store the request.
	 * @param bufferSize  Size of the buffer.
	 * @param velocity    Velocity to set.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setVelocityPrepareDataFrame(void*		buffer,
										   const size_t bufferSize,
										   const float	velocity);

	/**
	 * @brief Parses the response to a set velocity command.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @param velocity    Velocity that was attempted to be set (used for validation).
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setVelocityParseResponse(const void*	 buffer,
										const size_t bufferSize,
										const float	 velocity);

	/**
	 * @brief Prepares a frame to set the profile velocity.
	 *
	 * @param buffer      Pointer to the buffer to store the request.
	 * @param bufferSize  Size of the buffer.
	 * @param velocity    Profile velocity to set.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setProfileVelocityPrepareDataFrame(void*		   buffer,
												  const size_t bufferSize,
												  const float  velocity);

	/**
	 * @brief Parses the response to a set profile velocity command.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @param velocity    Velocity that was attempted to be set.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setProfileVelocityParseResponse(const void*	buffer,
											   const size_t bufferSize,
											   const float	velocity);

	/**
	 * @brief Prepares a frame to set the profile acceleration.
	 *
	 * @param buffer        Pointer to the buffer to store the request.
	 * @param bufferSize    Size of the buffer.
	 * @param acceleration  Acceleration value to set.
	 * @return              Error code indicating success or failure.
	 */
	errorMab_E setProfileAccelerationPrepareDataFrame(void*		   buffer,
													  const size_t bufferSize,
													  const float  acceleration);

	/**
	 * @brief Parses the response to a set profile acceleration command.
	 *
	 * @param buffer        Pointer to the received buffer.
	 * @param bufferSize    Size of the buffer.
	 * @param acceleration  Acceleration that was attempted to be set.
	 * @return              Error code indicating success or failure.
	 */
	errorMab_E setProfileAccelerationParseResponse(const void*	buffer,
												   const size_t bufferSize,
												   const float	acceleration);

	/**
	 * @brief Prepares a frame to set the profile deceleration.
	 *
	 * @param buffer        Pointer to the buffer to store the request.
	 * @param bufferSize    Size of the buffer.
	 * @param deceleration  Deceleration value to set.
	 * @return              Error code indicating success or failure.
	 */
	errorMab_E setProfileDecelerationPrepareDataFrame(void*		   buffer,
													  const size_t bufferSize,
													  const float  deceleration);

	/**
	 * @brief Parses the response to a set profile deceleration command.
	 *
	 * @param buffer        Pointer to the received buffer.
	 * @param bufferSize    Size of the buffer.
	 * @param deceleration  Deceleration that was attempted to be set.
	 * @return              Error code indicating success or failure.
	 */
	errorMab_E setProfileDecelerationParseResponse(const void*	buffer,
												   const size_t bufferSize,
												   const float	deceleration);

	/**
	 * @brief Prepares a frame to request the DC voltage of the system.
	 *
	 * @param buffer      Pointer to the buffer to store the request.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getDCVoltagePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the response containing the DC voltage value.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @param data        Pointer to store the voltage value.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getDCVoltageParseResponse(const void* buffer, const size_t bufferSize, float* data);

	/**
	 * @brief Prepares a data frame to send a shutdown command to the device.
	 *
	 * This function creates a CAN frame that instructs the device to prepare for shutdown.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateShutdownPrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the response to a shutdown command.
	 *
	 * This function checks if the device has acknowledged the shutdown command.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateShutdownParseResponse(const void* buffer, const size_t bufferSize);

	/**
	 * @brief Prepares a data frame to send an endable command to the device.
	 *
	 * This function creates a CAN frame that instructs the device to prepare for enabling.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateEnablePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the response to an endable command.
	 *
	 * This function checks if the device has acknowledged the endable command.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateEnableParseResponse(const void* buffer, const size_t bufferSize);

	/**
	 * @brief Prepares a data frame to send a disable command to the device.
	 *
	 * This function creates a CAN frame that instructs the device to prepare for disabling.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateDisablePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Parses the response to a disable command.
	 *
	 * This function checks if the device has acknowledged the disable command.
	 *
	 * @param buffer      Pointer to the received buffer.
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E setStateDisableParseResponse(const void* buffer, const size_t bufferSize);

	/**
	 * @brief Motor state containing the three main feedback values.
	 *
	 * All three registers are read in a single FDCAN frame (0x41) for efficiency.
	 */
	typedef struct
	{
		float mainEncoderPosition; /**< Main encoder position [rad] */
		float mainEncoderVelocity; /**< Main encoder velocity [rad/s] */
		float motorTorque;         /**< Output shaft torque [Nm] */
	} motorState_T;

	/**
	 * @brief Prepares a 20-byte FDCAN frame to read position, velocity and torque in one request.
	 *
	 * Packs REG_MAIN_ENCODER_POS (0x063), REG_MAIN_ENCODER_VEL (0x062) and
	 * REG_MOTOR_TORQUE (0x064) into a single READ_REGISTER_FDCAN (0x41) frame.
	 * The buffer must be at least 20 bytes.
	 *
	 * @param buffer      Pointer to the buffer where the frame data will be written.
	 * @param bufferSize  Size of the buffer in bytes (minimum 20).
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getMotorStatePrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Supported FDCAN data bit rates for bus scanning.
	 */
	typedef enum
	{
		FDCAN_DATA_BITRATE_1M = 1000000U,
		FDCAN_DATA_BITRATE_2M = 2000000U,
		FDCAN_DATA_BITRATE_5M = 5000000U,
		FDCAN_DATA_BITRATE_8M = 8000000U,
	} fdcanDataBitrate_E;

	/**
	 * @brief Callback used by scanDrivers to send a frame and receive the response.
	 *
	 * The implementation must transmit @p txBuf to the device with @p driverId, wait up to
	 * ~100 µs for a reply (per the MAB protocol spec), and copy the reply into @p rxBuf.
	 *
	 * @param driverId  CAN ID of the device to probe.
	 * @param txBuf     Frame to transmit.
	 * @param txSize    Number of bytes to transmit.
	 * @param rxBuf     Buffer to receive the response into.
	 * @param rxSize    Size of rxBuf in bytes.
	 * @return          OK if a response was received, ERROR_OPERATION_FAILED on timeout.
	 */
	typedef errorMab_E (*pingTxRxCb_F)(uint32_t    driverId,
	                                    const void* txBuf,
	                                    size_t      txSize,
	                                    void*       rxBuf,
	                                    size_t      rxSize);

	/**
	 * @brief Prepares a ping request frame (quickStatus read, 6 bytes).
	 *
	 * @param buffer      Pointer to the buffer (minimum PING_FRAME_SIZE bytes).
	 * @param bufferSize  Size of the buffer.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E pingPrepareDataFrame(void* buffer, const size_t bufferSize);

	/**
	 * @brief Validates a ping response frame.
	 *
	 * Checks that the frame ID is correct. The register value itself is not inspected —
	 * any valid response frame confirms the driver is present.
	 *
	 * @param buffer      Pointer to the received frame.
	 * @param bufferSize  Size of the received buffer.
	 * @return            OK if the response is valid, otherwise an error code.
	 */
	errorMab_E pingParseResponse(const void* buffer, const size_t bufferSize);

	/**
	 * @brief Scans the full CAN ID range and returns the IDs of all responding drivers.
	 *
	 * Iterates every CAN ID from CAN_MIN_ID to CAN_MAX_ID, sends a ping frame via
	 * @p txRxCb, and records IDs that reply. The caller is responsible for configuring
	 * the FDCAN peripheral to @p dataBitrate before calling this function.
	 *
	 * @param dataBitrate  Data bit rate the bus is operating at (used for validation).
	 * @param txRxCb       Callback that performs the CAN TX + RX for a single driver ID.
	 * @param foundIds     Array to store the IDs of responding drivers.
	 * @param maxIds       Capacity of foundIds (maximum number of IDs to store).
	 * @param foundCount   Output: number of responding drivers found.
	 * @return             OK on success, or an error code if arguments are invalid.
	 */
	errorMab_E scanDrivers(fdcanDataBitrate_E dataBitrate,
	                       pingTxRxCb_F       txRxCb,
	                       uint32_t*          foundIds,
	                       size_t             maxIds,
	                       size_t*            foundCount);

	/**
	 * @brief Parses the 20-byte FDCAN response to a motor state read request.
	 *
	 * Validates the frame ID and all three register IDs, then extracts the float
	 * values into the provided motorState_T struct.
	 *
	 * @param buffer      Pointer to the received CAN frame data.
	 * @param bufferSize  Size of the received buffer in bytes (minimum 20).
	 * @param state       Pointer to a motorState_T struct to store the result.
	 * @return            Error code indicating success or failure.
	 */
	errorMab_E getMotorStateParseResponse(const void*   buffer,
	                                      const size_t  bufferSize,
	                                      motorState_T* state);

#ifdef __cplusplus
}
#endif

#endif	// MAB_CAN_H
