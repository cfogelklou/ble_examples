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

#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "I2SCC26XX.h"

#include <string.h>

/* driverlib header files */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>
#include <driverlib/rom.h>
#include <driverlib/prcm.h>
#include <driverlib/i2s.h>

#ifndef LOG_ASSERT
static void log_assertionFailed(const char * const file, const int line){
    asm("nop");
    Assert_isTrue(false, 0);
}
#define LOG_ASSERT(expr) if (!expr) do{ log_assertionFailed(__FILE__, __LINE__);}while(0);
#endif


/* Internal functions */
static void I2SCC26XX_initHw(I2SCC26XX_Config * handle);
static bool I2SCC26XX_initIO(I2SCC26XX_Config * handle);
static void I2SCC26XX_hwiFxn (UArg arg);


I2SControlTable g_ControlTable;

/*
 *  ======== I2SCC26XX_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void I2SCC26XX_init(I2SCC26XX_Config * handle) {
    I2SCC26XX_Object         *object;

    /* Get the pointer to the object */
    object = handle->object;

    /* Mark the object as available */
    object->isOpen = false;

    /* Make sure struct in driverlib I2S driver is initialized */
    g_pControlTable = &g_ControlTable;
}

static void i2scc26xx_CallbackFxnDummy(I2SCC26XX_Config * handle, I2SCC26XX_StreamNotification *notification){

}

// Note: This file is MISSING from the driver. WTF TI?
bool I2SCC26XX_Params_init(
        I2SCC26XX_Params *params,
        I2SCC26XX_I2S_Params *i2sParams)
{
    bool rval = false;
    params->callbackFxn = i2scc26xx_CallbackFxnDummy;
    params->callbackFxn = i2sParams->callbackFxn;
    params->i32SampleRate = i2sParams->i32SampleRate;
    params->blockSizeSamples = i2sParams->blockSize;
    params->pvOutContBuffer = i2sParams->pvOutContBuffer;
    params->ui32OutConBufTotalSize = i2sParams->ui32OutConBufTotalSize;
    rval = true;
    return rval;
}


// Init DMA
static void i2scc26xx_initDmaControl(
        I2SCC26XXDmaControl * const pDmaCtrl,
        const int numChannels,
        const int dmaSize,
        const int sampleSizeBytes, // 2 or 3
        uint8_t * const pvOutContBuffer,
        uint32_t ui32OutConBufTotalBytes

)
{
    memset(pDmaCtrl, 0, sizeof(*pDmaCtrl));
    LOG_ASSERT(sampleSizeBytes <= 3 );
    LOG_ASSERT(sampleSizeBytes >= 2 );
    const int dmaSizeBytes = dmaSize*sampleSizeBytes;
    if ((numChannels > 0) && (dmaSizeBytes > 0) && (pvOutContBuffer) && (ui32OutConBufTotalBytes > dmaSizeBytes)) {
        int numBuffers = ui32OutConBufTotalBytes / dmaSizeBytes;
        pDmaCtrl->byteIdx = 0;
        pDmaCtrl->dmaSize = dmaSize;
        pDmaCtrl->dmaSizeBytes = dmaSizeBytes;
        pDmaCtrl->pContigAudioBuf = pvOutContBuffer;
        pDmaCtrl->contigBufSize = numBuffers * dmaSizeBytes;
    }
}


__STATIC_INLINE uint8_t * i2scc26xx_DmaCtrlInc(I2SCC26XXDmaControl * const pDma){
    pDma->byteIdx += pDma->dmaSizeBytes;
    if (pDma->byteIdx >= pDma->contigBufSize){
        pDma->byteIdx = 0;
    }
    return &pDma->pContigAudioBuf[pDma->byteIdx];
}

/*
 *  ======== I2SCC26XX_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
I2SCC26XX_Config * I2SCC26XX_open(I2SCC26XX_Config * handle, I2SCC26XX_Params *params) {
    I2SCC26XX_Object         * const object = handle->object;
    const I2SCC26XX_HWAttrs  * const hwAttrs = handle->hwAttrs;
    unsigned int             key  = Hwi_disable();

    /* Check if the I2S is open already with the base addr. */
    if (object->isOpen == true) {
        Log_warning1("I2S:(%p) already in use.", hwAttrs->baseAddr);
        goto error0;
    }

    /* Mark the handle as being used */
    object->isOpen = true;
    Hwi_restore(key);

    /* Initialize the I2S object */
    if (&object->params != params){
        memcpy( &object->params, params, sizeof(*params));
    }
    object->currentStream               = &object->currentStreamInst;
    object->currentStream->status       = I2SCC26XX_STREAM_IDLE;


    {
        // Find out how many channels are In and Out respectively
        I2SCC26XX_Params * const pP = &object->params;
        const uint32_t bytesPerSample = 2;
        uint8_t ui8TotalNumberOfChannelsIn = 0;
        uint8_t ui8TotalNumberOfChannelsOut = 0;
        switch (pP->audioPinCfg.bitFields.ad0Usage)
        {
            case I2SCC26XX_ADUsageInput:
                ui8TotalNumberOfChannelsIn += pP->audioPinCfg.bitFields.ad0NumOfChannels;
                break;
            case I2SCC26XX_ADUsageOutput:
                ui8TotalNumberOfChannelsOut += pP->audioPinCfg.bitFields.ad0NumOfChannels;
                break;
        }
        switch (pP->audioPinCfg.bitFields.ad1Usage)
        {
            case I2SCC26XX_ADUsageInput:
                ui8TotalNumberOfChannelsIn += pP->audioPinCfg.bitFields.ad1NumOfChannels;
                break;
            case I2SCC26XX_ADUsageOutput:
                ui8TotalNumberOfChannelsOut += pP->audioPinCfg.bitFields.ad1NumOfChannels;
                break;
        }

        i2scc26xx_initDmaControl(
                &object->inDma,
                ui8TotalNumberOfChannelsIn,
                object->params.blockSizeSamples,
                bytesPerSample,
                object->params.pvInContBuffer,
                object->params.ui32InConBufTotalSize
                );

        i2scc26xx_initDmaControl(
                &object->outDma,
                ui8TotalNumberOfChannelsOut,
                object->params.blockSizeSamples,
                bytesPerSample,
                object->params.pvOutContBuffer,
                object->params.ui32OutConBufTotalSize
                );

        /* Register power dependency - i.e. power up and enable clock for I2S. */
        Power_setDependency(hwAttrs->powerMngrId);
        /* Add dependency on uDMA to keep bus alive in IDLE */
        Power_setDependency(PowerCC26XX_PERIPH_UDMA);

        /* Configure IOs after hardware has been initialized so that IOs aren't */
        /* toggled unnecessary and make sure it was successful */
        if (!I2SCC26XX_initIO(handle)) {
            /* Trying to use I2S driver when some other driver or application has already allocated these pins, error! */
            Log_warning0("Could not allocate I2S pins, already in use.");
            // goto error2;
        }
    }

    /* Create the Hwi for this I2S peripheral. */
    {
        Hwi_Params hwiParams;
        Hwi_Params_init(&hwiParams);
        hwiParams.arg = (UArg) handle;
        hwiParams.priority = hwAttrs->intPriority;
        Hwi_construct(&(object->hwi), (int) hwAttrs->intNum, I2SCC26XX_hwiFxn, &hwiParams, NULL);
    }

    /* Check to see if a callback function was defined for async mode */
    Assert_isTrue(params->callbackFxn != NULL, NULL);

    Log_print1(Diags_USER2, "I2S:(%p) opened", hwAttrs->baseAddr);

    /* Register notification functions */
//    Power_registerNotify(&object->i2sPostObj, Power_AWAKE_STANDBY, (Fxn)i2sPostNotify, (UInt32)handle, NULL );

    return (handle);

error0:
    object->isOpen = false;
    Hwi_restore(key);

    /* Signal back to application that I2S driver was not succesfully opened */
    return (NULL);
}

/*
 *  ======== I2SCC26XX_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void I2SCC26XX_close(I2SCC26XX_Config * handle) {
    unsigned int                 key;
    I2SCC26XX_Object             *object;
    I2SCC26XX_HWAttrs const      *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    hwAttrs = handle->hwAttrs;
    object = handle->object;

    /* Deallocate pins */
    if (object->pinHandle){
        PIN_close(object->pinHandle);
    }

    /* Disable the I2S */
    I2SDisable(hwAttrs->baseAddr);

    /* Destroy the Hwi */
    Hwi_destruct(&(object->hwi));

    /* Release power dependency on I2S. */
    Power_releaseDependency(hwAttrs->powerMngrId);
    /* Release power dependency on UDMA. */
    Power_releaseDependency(PowerCC26XX_PERIPH_UDMA);

    /* Mark the module as available */
    key = Hwi_disable();
    object->isOpen = false;
    Hwi_restore(key);

    Log_print1(Diags_USER2, "I2S:(%p) closed", hwAttrs->baseAddr);
}


int I2SCC26XX_hwiFxnRuns = 0;
/*
 *  ======== I2SCC26XX_hwiFxn ========
 *  ISR for the I2S
 */
static void I2SCC26XX_hwiFxn (UArg arg) {
    int inInProgressIdx = -1;
    int outInProgressIdx = -1;
    I2SCC26XX_Config * const pConfig = (I2SCC26XX_Config *)arg;
    I2SCC26XX_Object  * const object = pConfig->object;
    I2SCC26XX_hwiFxnRuns++;
    if (true) {
        I2SCC26XX_HWAttrs * const hwAttrs = pConfig->hwAttrs;
        const uint32_t intStatus = I2SIntStatus(hwAttrs->baseAddr, true);

        /* Get the interrupt status of the I2S controller */
        I2SIntClear(hwAttrs->baseAddr, intStatus);


        if (intStatus & I2S_IRQMASK_AIF_DMA_IN) {

            if (object->currentStream->status != I2SCC26XX_STREAM_IDLE) {
                I2SCC26XXDmaControl * const pDma = &object->inDma;
                // Get the index of the bytes that are currently being output
                inInProgressIdx = pDma->byteIdx;
                // Set up double buffering.
                uint8_t *const pNextPtr = i2scc26xx_DmaCtrlInc(pDma);
                I2SPointerSet(hwAttrs->baseAddr, true, pNextPtr);
            }
        }

        if (intStatus & I2S_IRQMASK_AIF_DMA_OUT) {
            if (object->currentStream->status != I2SCC26XX_STREAM_IDLE) {
                I2SCC26XXDmaControl * const pDma = &object->outDma;

                // Get the index of the bytes that are currently being output
                outInProgressIdx = pDma->byteIdx;
                // Set up double buffering.
                uint8_t *const pNextPtr = i2scc26xx_DmaCtrlInc(pDma);
                I2SPointerSet(hwAttrs->baseAddr, false, pNextPtr);
            }
        }

        /* Error handling:
        * Overrun in the RX Fifo -> at least one sample in the shift
        * register has been discarded  */
        if (intStatus & I2S_IRQMASK_PTR_ERR) {
            /* disable the interrupt */
            I2SIntDisable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);
            /* Check if we are expecting this interrupt as part of stopping */
            if ( object->currentStream->status == I2SCC26XX_STREAM_IDLE ) {
            }
            else {
                asm(" NOP");
                object->currentStream->status = I2SCC26XX_STREAM_ERROR;
                /* Use a temporary stream pointer in case the callback function
                * attempts to perform another I2SCC26XX_bufferRequest call
                */
                /* Notify caller about availability of buffer */
                object->params.callbackFxn((I2SCC26XX_Config *)arg, object->currentStream);
                Log_print1(Diags_USER2, "I2S missing next pointer: (%p) !\n", hwAttrs->baseAddr);
            }
        }
        else if (intStatus & (I2S_INT_TIMEOUT | I2S_INT_BUS_ERR | I2S_INT_WCLK_ERR)) {
            asm(" NOP"); // Any other error
        }
    }

    // Call the callbacks depending on whether or not there have been in or out data.
    if ((inInProgressIdx >= 0) || (outInProgressIdx >= 0)){
        object->currentStreamInst.inCompletedBuf = object->currentStreamInst.outCompletedBuf = NULL;
        object->currentStreamInst.inCompletedBufBytes = object->currentStreamInst.outCompletedBufBytes = 0;
        if ((inInProgressIdx >= 0) && (object->inDma.pContigAudioBuf)){
            I2SCC26XXDmaControl * const pDma = &object->inDma;
            int completedByteIdx = inInProgressIdx - pDma->dmaSizeBytes;
            if (completedByteIdx < 0){
                completedByteIdx += pDma->contigBufSize;
            }
            // Get a pointer to the buffer that was just sent.
            object->currentStreamInst.inCompletedBuf = &pDma->pContigAudioBuf[completedByteIdx];
            object->currentStreamInst.inCompletedBufBytes = pDma->dmaSizeBytes;
        }
        if ((outInProgressIdx >= 0) && (object->outDma.pContigAudioBuf)){
            I2SCC26XXDmaControl * const pDma = &object->outDma;
            int completedByteIdx = outInProgressIdx - pDma->dmaSizeBytes;
            if (completedByteIdx < 0){
                completedByteIdx += pDma->contigBufSize;
            }
            // Get a pointer to the buffer that was just sent.
            object->currentStreamInst.outCompletedBuf = &pDma->pContigAudioBuf[completedByteIdx];
            object->currentStreamInst.outCompletedBufBytes = pDma->dmaSizeBytes;
        }
        object->params.callbackFxn(pConfig, &object->currentStreamInst);
    }
}


/*
 *  ======== I2SCC26XX_startStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool I2SCC26XX_startStream(I2SCC26XX_Config * handle) {
    unsigned int            key;
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttr*/
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if a transfer is in progress */
    key = Hwi_disable();
    if (object->currentStream->status != I2SCC26XX_STREAM_IDLE) {
        Hwi_restore(key);

        Log_error1("I2S:(%p) stream still in progress",
                ((I2SCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        /* Flag that the transfer failed to start */
        object->currentStream->status = I2SCC26XX_STREAM_ERROR;

        /* Transfer is in progress */
        return (false);
    }

    /* Make sure to flag that a stream is now active */
    object->currentStream->status = I2SCC26XX_STREAM_RUNNING;

    Hwi_restore(key);


    /* Set constraint during streaming */
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);

    /* Configure the hardware module */
    I2SCC26XX_initHw(handle);

    PRCMAudioClockDisable();
    PRCMLoadSet();

    /* Kick off clocks */
    PRCMAudioClockEnable();
    PRCMLoadSet();

    //TODO: Rename from I2S to AIF
    // Enable samplestamp
    //I2SSampleStampEnable(hwAttrs->baseAddr);

    /* Configure buffers */
    I2SBufferConfig(
            hwAttrs->baseAddr,
            (uint32_t )object->inDma.pContigAudioBuf,
            (uint32_t )object->outDma.pContigAudioBuf,
            object->params.blockSizeSamples,
            I2SCC26XX_DEFAULT_SAMPLE_STAMP_MOD);

    /* Enable the I2S module. This will set first buffer and DMA length */
    I2SEnable(hwAttrs->baseAddr);
    /* Second buffer is then set in hardware after DMA length is set */
    if (object->inDma.pContigAudioBuf) {
        uint8_t * const newPtr = i2scc26xx_DmaCtrlInc(&object->inDma);
        I2SPointerSet(hwAttrs->baseAddr, true, newPtr);
    }
    if (object->outDma.pContigAudioBuf) {
        uint8_t * const newPtr = i2scc26xx_DmaCtrlInc(&object->outDma);
        I2SPointerSet(hwAttrs->baseAddr, false, newPtr);
    }

    /* Enable the RX overrun interrupt in the I2S module */
    if (object->inDma.pContigAudioBuf) {
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_DMA_IN | I2S_INT_PTR_ERR);
    }
    if (object->outDma.pContigAudioBuf) {
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_DMA_OUT | I2S_INT_PTR_ERR | I2S_INT_ALL);
    }

    /* Clear internal pending interrupt flags */
    I2SIntClear(I2S0_BASE, I2S_INT_ALL);

    /* Configuring sample stamp generator will trigger the audio stream to start */
    I2SSampleStampConfigure(hwAttrs->baseAddr, (object->inDma.pContigAudioBuf) ? true : false, (object->outDma.pContigAudioBuf) ? true : false);

    /* Enable samplestamp */
    I2SSampleStampEnable(hwAttrs->baseAddr);

    /* Clear potential pending I2S interrupt to CM3 */
    Hwi_clearInterrupt(INT_I2S_IRQ);

    /* Enable I2S interrupt to CM3 */
    Hwi_enableInterrupt(INT_I2S_IRQ);

    return (true);
}

/*
 *  ======== I2SCC26XX_stopStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool I2SCC26XX_stopStream(I2SCC26XX_Config * handle) {
    I2SCC26XX_Object                    *object;
    I2SCC26XX_HWAttrs const             *hwAttrs;
    unsigned int                        key;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Check if a transfer is in progress */
    key = Hwi_disable();

    /* Check if there is an active stream */
    if (object->currentStream->status == I2SCC26XX_STREAM_IDLE) {
        Hwi_restore(key);
        return false;
    }

    /* Begin stopping sequence, if not stopped because of error */
    if (object->currentStream->status != I2SCC26XX_STREAM_ERROR) {

        /* Reenable the interrupt as it may have been disabled during an error*/
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);

        Hwi_restore(key);
    }

    /* restore HWI */
    Hwi_restore(key);

    /* Disable the I2S module */
    I2SDisable(hwAttrs->baseAddr);

    /* Disable and clear any pending interrupts */
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);
    I2SIntClear(hwAttrs->baseAddr, I2S_INT_ALL);
    Hwi_clearInterrupt(INT_I2S_IRQ);
    Hwi_disableInterrupt(INT_I2S_IRQ);


    /* Indicate we are done with this stream */
    object->currentStream->status = I2SCC26XX_STREAM_IDLE;

    Log_print2(Diags_USER2,"I2S:(%p) stream: %p stopped",
                            hwAttrs->baseAddr, (UArg)object->currentStream);

    /* Release constraint after streaming */
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);

    /* Stream was successfully stopped */
    return true;
}

//#define USE_TI_SETUP

/*
*  ======== I2SCC26XX_hwInit ========
*  This functions initializes the I2S hardware module.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static void I2SCC26XX_initHw(I2SCC26XX_Config * handle) {
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable I2S operation */
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);
    I2SDisable(hwAttrs->baseAddr);

    I2SAudioFormatConfigure(I2S0_BASE,
                            I2S_MEM_LENGTH_16     | // Sample size
                            I2S_POS_EDGE          | // Clock edge sampling
                            I2S_SINGLE_PHASE_FMT  | // Phase
                            I2S_WORD_LENGTH_16,     // Word length
                            1);

    I2SChannelConfigure(I2S0_BASE,
                        I2S_CHAN0_ACT | I2S_LINE_OUTPUT,  // single phase chan 0
                        0,              // null for chan 1
                        0);             // null for chan 2


    /* Turn on I2S clocks */
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2S);

    I2SClockConfigure(I2S0_BASE,
                      I2S_INT_WCLK |    // Clock source (internal)
                      I2S_INVERT_WCLK); // Clock polarity (not inverted)
    uint32_t ui32BitDiv = object->params.audioClkCfg.bclkDiv;
    uint32_t ui32WordDiv = object->params.audioClkCfg.wclkDiv;

    PRCMAudioClockConfigSetOverride(
            PRCM_WCLK_DUAL_PHASE | PRCM_WCLK_NEG_EDGE,
            128,
            ui32BitDiv,
            ui32WordDiv/2
            );


    // TODO: Replace this with Driverlib code
    HWREG(PRCM_BASE + PRCM_O_I2SBCLKSEL) = (object->params.audioClkCfg.bclkSource & PRCM_I2SBCLKSEL_SRC_M);
    /* Apply configuration */
    PRCMLoadSet();

    /* Disable I2S module interrupts */
//    uint32_t ui32IntFlags
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);

    /* Print the configuration */
    Log_print1(Diags_USER2, "I2S:(%p) Configured", hwAttrs->baseAddr);
}

// Internal utility function for setting mux setting in IOCFG register for pin
static void PINCC26XX_setIoCfgMux(PIN_Id pinId, int32_t nMux) {
    // Read in existing value in IOCFG register and update with supplied mux value
    if (nMux<0) {
        nMux = PINCC26XX_MUX_GPIO;
    }
    uint32_t dsCfg;
    dsCfg = HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId);
    dsCfg &= ~IOC_IOCFG0_PORT_ID_M;
    dsCfg |= nMux & IOC_IOCFG0_PORT_ID_M;
    HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId) = dsCfg;
}

/*
*  ======== I2SCC26XX_initIO ========
*  This functions initializes the I2S IOs.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static bool I2SCC26XX_initIO(I2SCC26XX_Config * handle) {

    I2SCC26XX_Object        * const object = handle->object;
    I2SCC26XX_HWAttrs const * const hwAttrs = handle->hwAttrs;

    /* Configure IOs */
    {
        PIN_Config              i2sPinTable[6];
        uint32_t                i = 0;
        /* Build local list of pins, allocate through PIN driver and map HW ports */
        if (object->params.audioPinCfg.bitFields.enableMclkPin) {
          i2sPinTable[i++] = hwAttrs->mclkPin | IOC_STD_OUTPUT;
        }
        if (object->params.audioPinCfg.bitFields.enableWclkPin) {
          i2sPinTable[i++] = hwAttrs->wclkPin | IOC_STD_OUTPUT;
        }
        if (object->params.audioPinCfg.bitFields.enableBclkPin) {
          i2sPinTable[i++] = hwAttrs->bclkPin | IOC_STD_OUTPUT;
        }
        if (object->params.audioPinCfg.bitFields.ad0Usage == I2SCC26XX_ADUsageInput) {
          i2sPinTable[i++] = hwAttrs->ad0Pin | PIN_INPUT_EN | PIN_NOPULL;
        }
        else if (object->params.audioPinCfg.bitFields.ad0Usage == I2SCC26XX_ADUsageOutput) {
          i2sPinTable[i++] = hwAttrs->ad0Pin | IOC_STD_OUTPUT;
        }
        if (object->params.audioPinCfg.bitFields.ad1Usage == I2SCC26XX_ADUsageInput) {
          i2sPinTable[i++] = hwAttrs->ad1Pin | IOC_STD_INPUT;
        }
        else if (object->params.audioPinCfg.bitFields.ad1Usage == I2SCC26XX_ADUsageOutput) {
          i2sPinTable[i++] = hwAttrs->ad1Pin | IOC_STD_OUTPUT;
        }
        i2sPinTable[i++] = PIN_TERMINATE;

        /* Open and assign pins through pin driver */
        object->pinHandle = PIN_open(&(object->pinState), i2sPinTable);
    }

    /* Get the pointer to the object and hwAttrs */

    /* Set IO muxing for the I2S pins */
    if (object->params.audioPinCfg.bitFields.enableMclkPin) {
      //PINCC26XX_setMux(object->pinHandle, hwAttrs->mclkPin,  IOC_PORT_MCU_I2S_MCLK);
      PINCC26XX_setIoCfgMux(hwAttrs->mclkPin, IOC_PORT_MCU_I2S_MCLK);
    }
    if (object->params.audioPinCfg.bitFields.enableWclkPin) {
      //PINCC26XX_setMux(object->pinHandle, hwAttrs->wclkPin,  IOC_PORT_MCU_I2S_WCLK);
      PINCC26XX_setIoCfgMux(hwAttrs->wclkPin, IOC_PORT_MCU_I2S_WCLK);
    }
    if (object->params.audioPinCfg.bitFields.enableBclkPin) {
      // PINCC26XX_setMux(object->pinHandle, hwAttrs->bclkPin,  IOC_PORT_MCU_I2S_BCLK);
      PINCC26XX_setIoCfgMux(hwAttrs->bclkPin, IOC_PORT_MCU_I2S_BCLK);
    }
    if (object->params.audioPinCfg.bitFields.ad0Usage != I2SCC26XX_ADUsageDisabled) {
      // PINCC26XX_setMux(object->pinHandle, hwAttrs->ad0Pin,  IOC_PORT_MCU_I2S_AD0);
      PINCC26XX_setIoCfgMux(hwAttrs->ad0Pin, IOC_PORT_MCU_I2S_AD0);
    }
    if (object->params.audioPinCfg.bitFields.ad1Usage != I2SCC26XX_ADUsageDisabled) {
      // PINCC26XX_setMux(object->pinHandle, hwAttrs->ad1Pin,  IOC_PORT_MCU_I2S_AD1);
      PINCC26XX_setIoCfgMux(hwAttrs->ad1Pin, IOC_PORT_MCU_I2S_AD1);
    }

    return (object->pinHandle) ? true : false;
}


/*
 *  ======== i2sPostNotify ========
 *  This functions is called to notify the I2S driver of an ongoing transition
 *  out of sleep mode.
 *
 *  @pre    Function assumes that the I2S handle (clientArg) is pointing to a
 *          hardware module which has already been opened.
 */
int i2sPostNotify(char eventType, uint32_t clientArg) {
    I2SCC26XX_Config * i2sHandle;

    /* Get the pointers to I2S objects */
    i2sHandle = (I2SCC26XX_Config *) clientArg;

    /* Reconfigure the hardware when returning from standby */
    I2SCC26XX_initHw(i2sHandle);

    return Power_NOTIFYDONE;
}

