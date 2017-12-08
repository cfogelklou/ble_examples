/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/** ============================================================================
 *  @file       I2SCC26XX.h
 *
 *  @brief      I2S driver implementation for a CC26XX I2S controller.
 *
 *  The I2S header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/i2s/I2SCC26XX.h>
 *  @endcode
 *
 * # Overview #
 * This driver is written specifically for the I2S module on CC26XX. The user
 * should be aware that although this module is called I2S it is a highly
 * configurable audio interface module. I2S is only one of many configurations.
 *
 * ## General Behavior #
 * Before using I2S on CC26XX:
 *   - The I2S driver object is initialized by calling I2SCC26XX_init(). This
 *     should be done in main before BIOS_start() is called.
 *   - The I2S system dependencies flags are set by calling I2SCC26XX_open().
 *     The driver is also taken. Prior to this the user should call
 *     I2SCC26XX_Params_init() to get the parameters to use in I2SCC26XX_open().
 *
 * While using I2S on CC26XX:
 *   - A stream is started, hardware configured and sleep prevented, when
 *     I2SCC26XX_startStream() is called. From now on callbacks are generated
 *     every time a buffer is ready; if callbacks are asked for.
 *   - Data is acquired by calling I2SCC26XX_requestBuffer(). Be sure to release
 *     buffer back to driver by calling I2SCC26XX_releaseBuffer().
 *   - The stream is stopped when I2SCC26XX_stopStream() is called. Note that
 *     this is a blocking call that will wait until the hardware has gracefully
 *     shut down. This will maximally take the same time that it takes to fill
 *     one buffer. The buffer size is configurable, so the time this function
 *     blocks is variable. System is allowed to sleep after
 *     I2SCC26XX_stopStream() is called.
 *
 * After I2S operation has ended:
 *   - Release system dependencies for I2S by calling I2SCC26XX_close().
 *
 *  ## Supported Functionality #
 * All possible configurations the I2S module can take are supported. However,
 * the user is encourage to use the I2SCC26XX_Params_init() function. Currently
 * the driver supports two configurations:
 *   - PDM
 *   - I2S (Note! Beta stage)
 *
 * ## Error handling #
 * Most APIs in this driver returns a boolean value indicating the success
 * or failure of the API. During streaming there is only one failure worth
 * discussing.
 * - Failure to provide new pointer to the I2S module (audio interface module)
 *      When stopping the stream this error is actually forced as it is part
 *      of the graceful shutdown sequence. It can however happen if the interrupt
 *      is not serviced in time, or there is no buffer available when serviced.
 *      - No buffer available:
 *              This error can happen if the caller does not request and release
 *              buffers fast enough to make sure at least one buffer is
 *              available for the driver. If this seems to be an intermittent
 *              unavoidable problem then it can be solved by providing a larger
 *              buffer to the driver.
 *      - Interrupt not serviced in time:
 *              This error can happen if interrupts are disabled, or another
 *              higher priority interrupt is being processed, for a duration
 *              longer than the time it takes to fill one buffer.
 * If the above mentioned error happens then the driver will notify the user
 * via the provided callback. It disables the pointer error interrupt, so that
 * it only generates this callback once. The user can now choose to process
 * the available buffers, before it must stop the stream. The user may, after
 * stopping the stream, restart the stream.
 *
 * ## Power Management #
 *  The I2SCC26XXDMA driver is setting a power constraint during stream to keep
 *  the device out of standby. When the stream has ended, the power
 *  constraint is released. The user is controlling the duration of the stream
 *  via calls to I2SCC26XX_startStream() and I2SCC26XX_stopStream().
 *  The following statements are valid:
 *    - After I2SCC26XX_open(): the device is still allowed to enter standby.
 *    - During I2SCC26XX_startStream(): the device cannot enter standby.
 *    - After I2SCC26XX_stopStream() succeeds: the device can enter standby.
 *    - If I2SCC26XX_close() is called: the device can enter standby.
 *
 * ## Supported Functions ##
 *  | API function               | Description                                          |
 *  |--------------------------- |------------------------------------------------------|
 *  | I2SCC26XX_init()           | Initialize I2S driver                                |
 *  | I2SCC26XX_Params_init()    | Get configuration specific parameters (e.g. PDM, I2S)|
 *  | I2SCC26XX_open()           | Set system dependencies, configure pins              |
 *  | I2SCC26XX_startStream()    | Initialize I2S HW, start stream and prevent standby  |
 *  | I2SCC26XX_stopStream()     | Stop stream and release standby hold                 |
 *  | I2SCC26XX_requestBuffer()  | Request a buffer from the driver                     |
 *  | I2SCC26XX_releaseBuffer()  | Release a buffer to the driver after processing it   |
 *  | I2SCC26XX_close()          | Disable I2S HW and release system dependencies       |
 *
 * ## Use Cases \anchor USE_CASES_I2S ##
 * ### PDM Mode #
 *  Receive data until told to stop.
 *  @code
 *  I2SCC26XX_StreamNotification pdmStream;
 *  I2SCC26XX_Config * i2sHandle;
 *  I2SCC26XX_BufferRequest bufferRequest;
 *  I2SCC26XX_BufferRelease bufferRelease;
 *  I2SCC26XX_Params i2sCC26XX_params;
 *  I2SCC26XX_PDM_Params i2sCC26XX_PDM_params = {
 *      I2SCC26XX_CALLBACK_MODE,
 *      BIOS_WAIT_FOREVER,
 *      NULL,
 *      PDM_BLOCK_SIZE_IN_SAMPLES,
 *      (void *) pdmContinuousBuffer,
 *      sizeof(pdmContinuousBuffer),
 *      (void *) pdmContMgtBuffer,
 *      sizeof(pdmContMgtBuffer),
 *      &pdmStream
 *  };
 *
 *  // Set callback function
 *  i2sCC26XX_PDM_params.callbackFxn = I2S_callbackFxn;
 *
 *  // Get PDM specific configuration
 *  I2SCC26XX_Params_init(&i2sCC26XX_params, I2SCC26XX_PDM, &i2sCC26XX_PDM_params);
 *
 *  // Then open the I2S
 *  I2SCC26XX_open(i2sHandle, &i2sCC26XX_params);
 *
 *  // Now you can start stream
 *  I2SCC26XX_startStream(i2sHandle);
 *
 *  // Wait for callback indicating buffers are ready. Typically this would
 *  // be in a thread waiting for a semaphore. The semaphore would be set in
 *  // the callback. In this case we're just assuming a volatile variable
 *  // bufferReady that is set in the callback
 *  while (!bufferReady);
 *  // Now request a buffer as it was indicated that one is available
 *  I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
 *  // Process the buffer, e.g. perform PDM2PCM conversion
 *  ...
 *  // Then release the buffer after it has been processed
 *  // Release PDM buffer
 *  bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
 *  bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
 *  I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
 *
 *  // Now go back to waiting for next buffer, or stop stream
 *
 *  // After processing all buffers one can stop the stream
 *  I2SCC26XX_stopStream(i2sHandle);
 *
 *  // Then process all remaining buffers
 *  while (I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest))
 *  {
 *      // Process the buffer, e.g. perform PDM2PCM conversion
 *      ...
 *      // Then release the buffer after it has been processed
 *      // Release PDM buffer
 *      bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
 *      bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
 *      I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
 *  }
 *
 *  // Then if we're really done, close the driver
 *  I2SCC26XX_close(i2sHandle);
 *
 *  @endcode
 *
 *  # Instrumentation #
 *  The I2SCC26XX driver has a debug mode that can be enabled compilation time.
 *  Use the flag I2S_DEBUG to enable this mode. In debug mode all buffers are
 *  labeled to indicate where in the process it is. It can have 6 different
 *  states:
 *
 *  Enum Queue Indicator        | Details               |
 *  --------------------------- | --------------------- |
 *  NODE_INDICATOR_NO_QUEUE     | not part of queue     |
 *  NODE_INDICATOR_AVAIL_QUEUE  | part of available     |
 *  NODE_INDICATOR_READY_QUEUE  | part of ready         |
 *  NODE_INDICATOR_NEXT         | next                  |
 *  NODE_INDICATOR_ACTIVE       | active                |
 *  NODE_INDICATOR_USER         | handled by user       |
 *
 *  # TODOs #
 *  * More optimized buffer management might be supported in later releases.
 *  * Complete ::I2SCC26XX_I2S_Params and enable I2S mode
 *  .
 *
 *  ============================================================================
 */

#ifndef ti_drivers_i2s_I2SCC26XX__include
#define ti_drivers_i2s_I2SCC26XX__include

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_types.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/*!
 * The following allows this header file to be included in an application file
 * which also includes ti/sysbios/hal/Hwi.h.
 */
#define ti_sysbios_family_arm_m3_Hwi__nolocalnames
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/*!
 *  @brief      Define to control debug mode
 *
 *  Production code should set this to xI2S_DEBUG. To enable debug mode
 *  rename the define to \b I2S_DEBUG.
 */
//#define I2S_DEBUG

/*!
 *  At least three elements must exist for good flow in driver
 */
#define I2SCC26XX_MIN_ALLOWED_QUEUE_SIZE            3

#ifndef I2SCC26XX_QUEUE_SIZE
#define    I2SCC26XX_QUEUE_SIZE                     7
#endif // !defined(I2SCC26XX_QUEUE_SIZE)

/*!
 *  PDM block overhead size in number of bytes --> sizeof(queueNodeI2S_t)
 */
#ifdef I2S_DEBUG
#define I2S_BLOCK_OVERHEAD_IN_BYTES             16
#else //I2S_DEBUG
#define I2S_BLOCK_OVERHEAD_IN_BYTES             12
#endif //I2S_DEBUG

/*! Return code when I2SCC26XX_control() was successful. */
#define I2SCC26XX_CMD_SUCCESS              0
/*! Return code when a I2S command or function is undefined/not-implemented. */
#define I2SCC26XX_CMD_UNDEFINED            -1
/*! Return code when I2SCC26XX_control() was unsuccessful. */
#define I2SCC26XX_CMD_NO_SUCCESS           -2

/*! Generic macro for disabled */
#define I2SCC26XX_GENERIC_DISABLED              0
/*! Generic macro for enabled */
#define I2SCC26XX_GENERIC_ENABLED               1


/*!
 *  @brief      Status codes that are set by the I2S driver.
 */
typedef enum I2SCC26XX_Status {
    I2SCC26XX_STREAM_IDLE = 0,  /*!< Idle mode. Stream not started */
    I2SCC26XX_STREAM_RUNNING,   /*!< Stream started, no buffer yet available */
    I2SCC26XX_STREAM_ERROR
} I2SCC26XX_Status;


/*!
 *  @brief
 *  Definitions for which I2SCC26XX buffers are requested.
 */
typedef enum I2SCC26XX_BuffersRequested {
    I2SCC26XX_BUFFER_NONE         = 0,    /*!< No I2SCC26XX buffer requested */
    I2SCC26XX_BUFFER_IN           = 1,    /*!< I2SCC26XX buffer In requested */
    I2SCC26XX_BUFFER_OUT          = 2,    /*!< I2SCC26XX buffer Out requested */
    I2SCC26XX_BUFFER_IN_AND_OUT   = 3,    /*!< I2SCC26XX buffer In and Out requested */
} I2SCC26XX_BuffersRequested;

/*!
 *  We don't use sample stamp generator, but we can't start without
 *  enabling it and configuring the trigger. This is because the
 *  trigger also starts the audio stream
 *  Since we don't use it we keep the word period at its max 2^16
 *  For the driverlib which runs a modulo on the word period we can
 *  set the modulo to 0xFFFF to avoid issues with division by zero.
 */
#define I2SCC26XX_DEFAULT_SAMPLE_STAMP_MOD      0x0000FFFF

/*!
 *  Definitions for different I2S Word Clock phase settings.
 *
 *  Defines how WCLK division ratio is calculated to generate different
 *  duty cycles.
 *  \sa I2SWCLKDIV.WDIV
 *
 *  Macro                                       | Value |
 *  ------------------------------------------- | ------|
 *  I2SCC26XX_WordClockPhase_Single             | 0     |
 *  I2SCC26XX_WordClockPhase_Dual               | 1     |
 *  I2SCC26XX_WordClockPhase_UserDefined        | 2     |
 */
#define I2SCC26XX_WordClockPhase_Single         0
/*!
 *  \sa I2SCC26XX_WordClockPhase_Single
 */
#define I2SCC26XX_WordClockPhase_Dual           1
/*!
 *  \sa I2SCC26XX_WordClockPhase_Single
 */
#define I2SCC26XX_WordClockPhase_UserDefined    2

/*!
 *  Definitions to set sample edge
 *
 *  Macro                               | Value |
 *  ----------------------------------- | ------|
 *  I2SCC26XX_SampleEdge_Negative       | 0     |
 *  I2SCC26XX_SampleEdge_Postive        | 1     |
 */
#define I2SCC26XX_SampleEdge_Negative   0
/*!
 *  \sa I2SCC26XX_SampleEdge_Negative
 */
#define I2SCC26XX_SampleEdge_Postive    1

/*!
 *  Definitions different I2S Word Clock source settings.
 *
 *  Macro                               | Value |
 *  ----------------------------------- | ------|
 *  I2SCC26XX_WordClockSource_Ext       | 0     |
 *  I2SCC26XX_WordClockSource_Int       | 1     |
 */
#define I2SCC26XX_WordClockSource_Ext       1
/*!
 *  \sa I2SCC26XX_WordClockSource_Ext
 */
#define I2SCC26XX_WordClockSource_Int       2

/*!
 *  Definitions different I2S Bit Clock source settings.
 *
 *  Macro                               | Value |
 *  ----------------------------------- | ------|
 *  I2SCC26XX_BitClockSource_Ext        | 0     |
 *  I2SCC26XX_BitClockSource_Int        | 1     |
 */
#define I2SCC26XX_BitClockSource_Ext       0
/*!
 *  \sa I2SCC26XX_BitClockSource_Ext
 */
#define I2SCC26XX_BitClockSource_Int       1

/*!
 *  Definitions to either invert I2S word or bit clock or not
 *
 *  Macro                               | Value |
 *  ----------------------------------- | ------|
 *  I2SCC26XX_ClockSource_Normal        | 0     |
 *  I2SCC26XX_ClockSource_Inverted      | 1     |
 */
#define I2SCC26XX_ClockSource_Normal    0
/*!
 *  \sa I2SCC26XX_ClockSource_Normal
 */
#define I2SCC26XX_ClockSource_Inverted  1

/*!
 *  I2SCC26XX Audio Data Pin Usage.
 *
 *  Macro                       | Details       |
 *  --------------------------- | --------------|
 *  I2SCC26XX_ADUsageDisabled   | Disabled      |
 *  I2SCC26XX_ADUsageInput      | Input         |
 *  I2SCC26XX_ADUsageOutput     | Output        |
 */
#define I2SCC26XX_ADUsageDisabled       0
/*!
 *  \sa I2SCC26XX_ADUsageDisabled
 */
#define I2SCC26XX_ADUsageInput          1
/*!
 *  \sa I2SCC26XX_ADUsageDisabled
 */
#define I2SCC26XX_ADUsageOutput         2

/*!
 *  I2SCC26XX Audio Channel Masks.
 *
 *  Macro                       | Value         | Usage                         |
 *  --------------------------- | --------------|-------------------------------|
 *  I2SCC26XX_CHAN0_ACT         | 0x00000001    | OR in to enable channel 0     |
 *  I2SCC26XX_CHAN1_ACT         | 0x00000002    | OR in to enable channel 1     |
 *  I2SCC26XX_CHAN2_ACT         | 0x00000004    | OR in to enable channel 2     |
 *  I2SCC26XX_CHAN3_ACT         | 0x00000008    | OR in to enable channel 3     |
 *  I2SCC26XX_CHAN4_ACT         | 0x00000010    | OR in to enable channel 4     |
 *  I2SCC26XX_CHAN5_ACT         | 0x00000020    | OR in to enable channel 5     |
 *  I2SCC26XX_CHAN6_ACT         | 0x00000040    | OR in to enable channel 6     |
 *  I2SCC26XX_CHAN7_ACT         | 0x00000080    | OR in to enable channel 7     |
 *  I2SCC26XX_MONO_MODE         | 0x00000001    | Use to set Mono mode          |
 *  I2SCC26XX_STEREO_MODE       | 0x00000003    | Use to set Stereo mode        |
 *  I2SCC26XX_DISABLED_MODE     | 0x00000000    | Use to disable                |
 *  I2SCC26XX_CHAN_CFG_MASK     | 0x000000FF    | Use to mask out invalid       |
 */
#define I2SCC26XX_CHAN0_ACT           0x00000001
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN1_ACT           0x00000002
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN2_ACT           0x00000004
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN3_ACT           0x00000008
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN4_ACT           0x00000010
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN5_ACT           0x00000020
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN6_ACT           0x00000040
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN7_ACT           0x00000080
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_MONO_MODE           0x00000001
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_STEREO_MODE         0x00000003
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_DISABLED_MODE       0x00000000
/*! \sa I2SCC26XX_CHAN0_ACT */
#define I2SCC26XX_CHAN_CFG_MASK       0x000000FF

/*!
 *  I2SCC26XX data word length is used to determine how bits to transfer per word.
 *
 *  Macro                       | Value | Usage                                 |
 *  --------------------------- | ------|---------------------------------------|
 *  I2SCC26XX_WordLengthMin     | 8     | Minimum transfer length is 8 bits     |
 *  I2SCC26XX_WordLength16      | 16    | A typical transfer length is 16 bits  |
 *  I2SCC26XX_WordLengthMax     | 24    | Maximum transfer length is 24 bits    |
 */
#define I2SCC26XX_WordLengthMin         8
/*! \sa I2SCC26XX_WordLengthMin */
#define I2SCC26XX_WordLength16          16
/*! \sa I2SCC26XX_WordLengthMin */
#define I2SCC26XX_WordLengthMax         24

/*!
 *  I2SCC26XX Phase is set to select Dual or Single phase format
 *
 *  Macro                       | Value |
 *  --------------------------- | ------|
 *  I2SCC26XX_SinglePhase       | 0     |
 *  I2SCC26XX_DualPhase         | 1     |
 */
#define I2SCC26XX_SinglePhase           0
/*! \sa I2SCC26XX_SinglePhase */
#define I2SCC26XX_DualPhase             1

/*!
 *  I2SCC26XX Sample Edge is set to control what edge to sample and clock out
 *  data on.
 *
 *  Macro                       | Value |
 *  --------------------------- | ------|
 *  I2SCC26XX_NegativeEdge      | 0     |
 *  I2SCC26XX_PositiveEdge      | 1     |
 */
#define I2SCC26XX_NegativeEdge          0
/*! \sa I2SCC26XX_NegativeEdge */
#define I2SCC26XX_PositiveEdge          1

/*!
 *  I2SCC26XX data word size is used to determine how to configure the
 *  I2S data transfers to/from memory.
 *
 *  Macro                       | Value | Usage                         |
 *  --------------------------- | ------|-------------------------------|
 *  I2SCC26XX_MemLen16bit       | 0     | sample 16 bits per word       |
 *  I2SCC26XX_MemLen24bit       | 1     | sample 24 bits per word       |
 *
 *  I2SCC26XX_MemLen16bit: sample 16 bits per word
 *  I2SCC26XX_MemLen24bit: sample 24 bits per word
 */
#define I2SCC26XX_MemLen16bit  0
/*! \sa I2SCC26XX_MemLen16bit */
#define I2SCC26XX_MemLen24bit  1

/*!
 *  I2SCC26XX Data Delay, which translates into format (LJF, I2S/DSP, RJF).
 *
 *  This field can be set to any 8 bit value. The macros are just defined
 *  for convenience. Left justified mode means that sampling should start
 *  immediately. For right justified mode the data delay depends on how
 *  many samples should be taken per word. It is an alignment.
 *
 *  I2S is a special mode that defines that no sample occur on first edge,
 *  hence there is one period data delay.
 *
 *  Macro                       | Value | Usage                         |
 *  --------------------------- | ------|-------------------------------|
 *  I2SCC26XX_FormatLJF         | 0     | no data delay                 |
 *  I2SCC26XX_FormatI2SandDSP   | 1     | one period data delay         |
 *  I2SCC26XX_FormatRJFmin      | 2     | two periods data delay        |
 *  I2SCC26XX_FormatRJFmax      | 255   | 255 periods data delay        |
 */
#define I2SCC26XX_FormatLJF                     0
/*! \sa I2SCC26XX_FormatLJF */
#define I2SCC26XX_FormatI2SandDSP               1
/*! \sa I2SCC26XX_FormatLJF */
#define I2SCC26XX_FormatRJFmin                  2
/*! \sa I2SCC26XX_FormatLJF */
#define I2SCC26XX_FormatRJFmax                  255

/*! Number of samples (16 or 24 bits) per queue element buffer */
typedef uint32_t I2SCC26XX_TransferSize;

/*!
 *  @brief
 *  The I2SCC26XX_Config structure contains a set of pointers used to characterize
 *  the I2SCC26XX driver implementation.
 */
struct I2SCC26XX_Object; // Forward declaration
struct I2SCC26XX_HWAttrs;// Forward declaration
typedef struct I2SCC26XX_Config {
    struct I2SCC26XX_Object *object;
    struct I2SCC26XX_HWAttrs *hwAttrs;
} I2SCC26XX_Config;

extern const I2SCC26XX_Config I2SCC26XX_config[];

/*!
 *  @brief  I2SCC26XX Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CC26xxWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - inc/hw_ints.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const I2SCC26XX_HWAttrs i2sCC26XXHWAttrs = {
 *      {
 *          I2S0_BASE,
 *          INT_I2S,
 *          PERIPH_I2S,
 *          Board_I2S_MCLK,
 *          Board_I2S_BCLK,
 *          Board_I2S_WCLK,
 *          Board_I2S_ADI,
 *          Board_I2S_ADO
 *      },
 *  };
 *  @endcode
 */
typedef struct I2SCC26XX_HWAttrs {
    /*! I2S Peripheral's base address */
    uint32_t         baseAddr;
    /*! I2S Peripheral's interrupt vector */
    uint8_t          intNum;
    /*! I2S Peripheral's interrupt priority */
    uint8_t          intPriority;
    /*! I2S Peripheral's power manager ID */
    PowerCC26XX_Resource   powerMngrId;
    /*! I2S MCLK pin */
    PIN_Id           mclkPin;
    /*! I2S BCLK pin */
    PIN_Id           bclkPin;
    /*! I2S WCLK pin */
    PIN_Id           wclkPin;
    /*! I2S AD0 pin */
    PIN_Id           ad0Pin;
    /*! I2S AD1 pin */
    PIN_Id           ad1Pin;
} I2SCC26XX_HWAttrs;

/*!
 *  @brief  I2SCC26XX Audio Clock configuration
 *
 *  These fields are used by the driver to set up the I2S module
 *
 *  A sample structure is shown below (single PDM microphone):
 *  @code
 *  const I2SCC26XX_AudioClockConfig i2sCC26XXobjects[] = {
 *          16, // Word clock division
 *          I2SCC26XX_SampleEdge_Negative,
 *          I2SCC26XX_WordClockPhase_Dual,
 *          I2SCC26XX_ClockSource_Inverted,
 *          I2SCC26XX_WordClockSource_Int,
 *          47, // Bit clock division
 *          0, // Reserved
 *          I2SCC26XX_BitClockSource_Int
 *          6, // Master clock division
 *  };
 *  @endcode
 */
typedef struct I2SCC26XX_AudioClockConfig {
    /*! I2S Word Clock divider override */
    uint16_t    wclkDiv;
    /*! I2S Bit Clock divider override */
    uint16_t    bclkDiv:10;
    /*! I2S Bit Clock source (I2SCC26XX_BitClockSource_Ext or I2SCC26XX_BitClockSource_Int) */
    uint16_t    bclkSource:1;
} I2SCC26XX_AudioClockConfig;

/*!
 *  @brief  I2SCC26XX Audio Pin configuration
 *
 *  These fields are used by the driver to set up the I2S module
 *
 *  A sample structure is shown below (single PDM microphone):
 *  @code
 *  const I2SCC26XX_AudioPinConfig i2sCC26XXobjects[] = {
 *          I2SCC26XX_ADUsageDisabled,
 *          0,
 *          0,
 *          0,
 *          0,
 *          I2SCC26XX_ADUsageInput,
 *          0,
 *          1,
 *          2,
 *          I2S_MONO_MODE
 *  };
 *  @endcode
 */
typedef union I2SCC26XX_AudioPinConfig {
  /*! Can be used to set pin configurations in DriverLib*/
  struct {
    /*! Field that can be used to set pin configuration in DriverLib */
    uint16_t    ad1;
    /*! Field that can be used to set pin configuration in DriverLib */
    uint16_t    ad0;
  } driverLibParams;
  /*! Used to configure various aspects of the I2S hardware during initialisation */
  struct {
    /*! I2S AD1 usage (0: Disabled, 1: Input, 2: Output) */
    uint8_t     ad1Usage:2;
    /*! I2S Enable Master clock output on pin (0: Disabled, 1: Enabled) */
    uint8_t     enableMclkPin:1;
    /*! Reserved bit field */
    uint8_t     reserved:1;
    /*! I2S AD1 number of channels (1 - 8). !Must match channel mask */
    uint8_t     ad1NumOfChannels:4;
    /*! I2S AD1 Channel Mask bitwise 0:Disabled, 1:Enabled) E.g. Mono: 0x01, Stereo: 0x03 */
    uint8_t     ad1ChannelMask;
    /*! I2S AD0 usage (0: Disabled, 1: Input, 2: Output) */
    uint8_t     ad0Usage:2;
    /*! I2S Enable Word clock output on pin (0: Disabled, 1: Enabled) */
    uint8_t     enableWclkPin:1;
    /*! I2S Enable Bit clock output on pin (0: Disabled, 1: Enabled) */
    uint8_t     enableBclkPin:1;
    /*! I2S AD0 number of channels (1 - 8). !Must match channel mask*/
    uint8_t     ad0NumOfChannels:4;
    /*! I2S AD0 Channel Mask bitwise(0:Disabled, 1:Enabled) E.g. Mono: 0x01, Stereo: 0x03 */
    uint8_t     ad0ChannelMask;
  } bitFields;
} I2SCC26XX_AudioPinConfig;
/*! Mask to use with I2SCC26XX_AudioPinConfig.driverLibParams when calling
 * DriverLib.
 */
#define I2SCC26XX_DIR_CHA_M     (I2S_LINE_MASK | I2S_CHAN_CFG_MASK)

/*!
 *  @brief  I2SCC26XX Hardware configuration
 *
 *  These fields are used by the driver to set up the I2S module
 *
 *  A sample structure is shown below (single PDM microphone):
 *  @code
 *  const I2SCC26XX_AudioFormatConfig i2sCC26XXobjects[] = {
 *          I2SCC26XX_WordLength16,
 *          I2SCC26XX_PositiveEdge,
 *          I2SCC26XX_DualPhase,
 *          I2SCC26XX_MemLen16bit,
 *          I2SCC26XX_FormatI2SandDSP
 *  };
 *  @endcode
 */

/*!
 *  @brief
 *  A ::I2SCC26XX_StreamNotification data structure is used with I2SCC26XX_CallbackFxn().
 *  Provides notification about available buffers and potential errors
 */
typedef struct I2SCC26XX_StreamNotification {
    void      *arg;             /*!< Argument to be passed to the callback function */
    I2SCC26XX_Status status;    /*!< Status code set by I2SCC26XX driver */
    uint8_t   *outCompletedBuf;
    int        outCompletedBufBytes;
    uint8_t   *inCompletedBuf;
    int        inCompletedBufBytes;
} I2SCC26XX_StreamNotification;


/*!
 *  @brief      The definition of a callback function used when wakeup on
 *              chip select is enabled
 *
 *  @param      I2SCC26XX_Config *          I2SCC26XX_Config *
 */
typedef void        (*I2SCC26XX_CallbackFxn) (
        I2SCC26XX_Config * handle,
        I2SCC26XX_StreamNotification *notification);

/*!
 *  @brief
 *  I2SCC26XX I2S Parameters are used to with the I2SCC26XX_Params_init() call.
 *
 *  @sa     I2SCC26XX_Params_init
 */
typedef struct I2SCC26XX_I2S_Params {
    I2SCC26XX_CallbackFxn       callbackFxn;            /*!< Callback function pointer */
    int32_t                     i32SampleRate;          /*!< I2S bit clock frequency in Hz. If negative, or one of I2S_SAMPLE_RATE_16K/_24K/_32K/_48K then use user configured clock division.*/
    I2SCC26XX_TransferSize      blockSize;              /*!< I2S DMA transfer size in number of samples. Each
                                                         * sample consumes either 16 or 24 bits per channel,
                                                         * set by ::I2SCC26XX_AudioFormatConfig.memLen. Number
                                                         * of channels are set in
                                                         * ::I2SCC26XX_AudioPinConfig.ad0NumOfChannels and
                                                         * ::I2SCC26XX_AudioPinConfig.ad1NumOfChannels*/

    /* I2S stream variables */
    void                        *pvOutContBuffer;          /*!< Pointer to consecutive buffer in memory. Driver
                                                         * will chunk it into the queue. Make sure to provide
                                                         * correct buffer size ::I2SCC26XX_I2S_Params.ui32conBufTotalSize*/
    uint32_t                    ui32OutConBufTotalSize;    /*!< Size of consecutive buffer must match total
                                                         * available sample size: (wanted number of blocks) *
                                                         * (number of samples per block) * (samplesize(in bytes)) *
                                                         * number of channels*/
} I2SCC26XX_I2S_Params;

/*!
 *  @brief
 *  I2SCC26XX Parameters are used to with the I2SCC26XX_open() call. Default values for
 *  these parameters are set using I2SCC26XX_Params_init().
 *
 *  @sa     I2SCC26XX_Params_init
 */
typedef struct I2SCC26XX_Params {
    /* I2S control variables */
    I2SCC26XX_CallbackFxn       callbackFxn;            /*!< Callback function pointer */
    int32_t                     i32SampleRate;          /*!< I2S bit clock frequency in Hz. If negative, or one of I2S_SAMPLE_RATE_16K/_24K/_32K/_48K then use user configured clock division.*/
    I2SCC26XX_AudioClockConfig  audioClkCfg;            /*!< I2S clock division override and clock config*/
    I2SCC26XX_AudioPinConfig    audioPinCfg;            /*!< I2S pin configuration*/
    I2SCC26XX_TransferSize      blockSizeSamples;              /*!< I2S DMA transfer size in number of samples. Each sample consumes either 16 or 24 bits per channel, set by ::I2SCC26XX_AudioFormatConfig.memLen. Number of channels are set in ::I2SCC26XX_AudioPinConfig.adXNumOfChannels*/

    /* I2S stream variables */
    /* I2S stream variables */
    uint8_t                     *pvOutContBuffer;          /*!< Pointer to consecutive buffer in memory. Driver will chunk it into the queue*/
    uint32_t                     ui32OutConBufTotalSize;    /*!< Size of consecutive buffer must match total available sample size: block * number of samples * samplesize(in bytes) * number of channels*/
    uint8_t                     *pvInContBuffer;          /*!< Pointer to consecutive buffer in memory. Driver will chunk it into the queue*/
    uint32_t                     ui32InConBufTotalSize;    /*!< Size of consecutive buffer must match total available sample size: block * number of samples * samplesize(in bytes) * number of channels*/

    void                        *custom;                /*!< Custom argument used by driver implementation */
} I2SCC26XX_Params;

typedef struct I2SCC26XXDmaControlTag {
    uint8_t *pContigAudioBuf;
    int dmaSize;
    int dmaSizeBytes;
    int contigBufSize;
    int byteIdx;
} I2SCC26XXDmaControl;

/*!
 *  @brief  I2SCC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct I2SCC26XX_Object {
    I2SCC26XX_Params            params;

    I2SCC26XX_StreamNotification currentStreamInst;        /*!< Ptr to information about the current transaction*/
    I2SCC26XX_StreamNotification *currentStream;        /*!< Ptr to information about the current transaction*/

    // Private stuffs
    I2SCC26XXDmaControl         outDma;
    I2SCC26XXDmaControl         inDma;

    /* I2S SYS/BIOS objects */
    ti_sysbios_family_arm_m3_Hwi_Struct hwi;            /*!< Hwi object handle */

    /* PIN driver state object and handle */
    PIN_State                   pinState;               /*!< PIN driver state object */
    PIN_Handle                  pinHandle;              /*!< PIN driver handle */

    bool                        isOpen;                 /*!< Has the object been opened */
} I2SCC26XX_Object;

/*!
 *  @brief I2S CC26XX initialization
 *
 *  @param handle  A I2SCC26XX_Config *
 *
 */
extern void I2SCC26XX_init(I2SCC26XX_Config * handle);

/*!
 *  @brief  Function to set initialization parameters for the audio interface.
 *
 *  The parameter \b mode specifies which mode the audio interface module will
 *  operate. \b modeSpecificParams must point to a struct that matches the
 *  selected mode. This is because this function will type-cast the void* based
 *  on the selected mode.
 *
 *  \b params is filled correctly by this function. Its memory must be allocated
 *  by the caller. Note however that it only has to live past the call to
 *  I2SCC26XX_open().
 *
 *  @param params       Pointer to allocated structure to hold parameters to be used in I2SCC26XX_open
 *  @i2sParams Pointer to parameters that are specific to the requested configuration
 *
 */
extern bool I2SCC26XX_Params_init(
        I2SCC26XX_Params *params,
        I2SCC26XX_I2S_Params *i2sParams);

/*!
 *  @brief  Function to open a given CC26XX I2S peripheral specified by the
 *          I2S handle.
 *
 *  The function will set a dependency on its power domain, i.e. power up the
 *  module and enable the clock. The IOs are allocated. The I2S will not be
 *  enabled.
 *
 *  \b params must point a correctly initialized I2SCC26XX_Params struct.
 *  I2SCC26XX_Params_init is a convenience function to initialize the struct
 *  correctly.
 *
 *  @pre    I2S controller has been initialized
 *
 *  @param  handle        A I2SCC26XX_Config *
 *
 *  @param  params        Pointer to a parameter block, if NULL it will use
 *                        default values
 *
 *  @return A I2SCC26XX_Config * on success or a NULL on an error or if it has been
 *          already opened
 *
 *  @sa     I2SCC26XX_close()
 */
extern I2SCC26XX_Config *  I2SCC26XX_open(I2SCC26XX_Config * handle, I2SCC26XX_Params *params);

/*!
 *  @brief  Function to close a given CC26XX I2S peripheral specified by the
 *          I2S handle.
 *
 *  Will disable the I2S, disable all I2S interrupts and release the
 *  dependency on the corresponding power domain. It will also destroy all
 *  semaphores and queues that have been used.
 *
 *  @pre    I2SCC26XX_open() has to be called first.
 *
 *  @param  handle  A I2SCC26XX_Config * returned from I2SCC26XX_open()
 *
 *  @sa     I2SCC26XX_open
 */
extern void I2SCC26XX_close(I2SCC26XX_Config * handle);

/*!
 *  @brief  Function for starting an I2S interface.
 *
 *  Calling this function will prevent the device from sleeping, as the
 *  stream is continuous and thus require power to the audio interface module
 *  from start to end.
 *  This function will configure all hardware registers that does not have
 *  retention. Hence, one may call I2SCC26XX_open then go to sleep before this
 *  function is called.
 *
 *  If non-blocking mode is selected then the I2SCC26XX module will begin
 *  calling the provided callback function every time a new buffer is ready.
 *
 *  @pre    I2SCC26XX_open() has to be called first.
 *
 *  @param  handle An I2S handle returned from I2SCC26XX_open()
 *
 *  @return True if transfer is successful and false if not
 *
 *  @sa     I2SCC26XX_open(), I2SCC26XX_stopStream()
 */
extern bool I2SCC26XX_startStream(I2SCC26XX_Config * handle);

/*!
 *  @brief  Function for stopping an I2S interface.
 *
 *  This function will initiate the shut down sequence. The audio interface
 *  module is designed with a graceful shutdown. What this means is that the
 *  current buffer is filled before stopping. This function will block while
 *  the last buffer is completing. The maximum blocking delay is a function
 *  of the configured DMA transfer size, and the word clock rate.
 *
 *  When this function returns it is recommended to complete processing of
 *  all pending ready buffers. If the caller is not interested in the last
 *  audio data it may simply call I2SCC26XX_requestBuffer() and
 *  I2SCC26XX_releaseBuffer() in a loop until I2SCC26XX_requestBuffer() returns
 *  false.
 *
 *  Will disable the I2S, disable all I2S interrupts and release the
 *  dependency on the corresponding power domain.
 *
 *  @pre    I2SCC26XX_startStream() has to be called first.
 *
 *  @param  handle An I2S handle returned from I2SCC26XX_open()
 *
 *  @return True if stream stopped successfully and false if not
 *
 *  @sa     I2SCC26XX_open(), I2SCC26XX_startStream()
 */
extern bool I2SCC26XX_stopStream(I2SCC26XX_Config * handle);


/* Do not interfere with the app if they include the family Hwi module */
#undef ti_sysbios_family_arm_m3_Hwi__nolocalnames

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_i2s_I2SCC26XX__include */
