#include "NanostackRfPhyKw41z.h"
#include "ns_types.h"
#include "platform/arm_hal_interrupt.h"
#include "nanostack/platform/arm_hal_phy.h"
#include <string.h>
#include "rtos.h"
#include "fsl_xcvr.h"
#include "PhyTypes.h"

/* PHY states */
typedef enum {
  gIdle_c,
  gRX_c,
  gTX_c,
  gCCA_c,
  gTR_c,
  gCCCA_c,
}phySeqState_t;

typedef enum{
    gPhyPwrIdle_c,
    gPhyPwrAutodoze_c,
    gPhyPwrDoze_c,
    gPhyPwrHibernate_c,
    gPhyPwrDSM_c,
    gPhyPwrReset_c
}phyPwrMode_t;

/* PHY channel state */
enum {
  gChannelIdle_c,
  gChannelBusy_c
};

/* PANCORDNTR bit in PP */
enum {
  gMacRole_DeviceOrCoord_c,
  gMacRole_PanCoord_c
};

/* Cca types */
enum {
  gCcaED_c,            /* energy detect - CCA bit not active, not to be used for T and CCCA sequences */
  gCcaCCA_MODE1_c,     /* energy detect - CCA bit ACTIVE */
  gCcaCCA_MODE2_c,     /* 802.15.4 compliant signal detect - CCA bit ACTIVE */
  gCcaCCA_MODE3_c,     /* 802.15.4 compliant signal detect and energy detect - CCA bit ACTIVE */
  gInvalidCcaType_c    /* illegal type */
};

enum {
  gNormalCca_c,
  gContinuousCca_c
};

static phySeqState_t  mPhyState = gIdle_c;
static phyPwrMode_t mPhyPwrState = gPhyPwrIdle_c;
static uint8_t MAC_address[8] = {1, 2, 3, 4, 5, 6, 7, 8};
static NanostackRfPhyKw41z *rf = NULL;
static Thread irq_thread(osPriorityRealtime, 1024);
static phy_device_driver_s device_driver;
static int8_t  rf_radio_driver_id = -1;
static uint8_t  mac_tx_handle = 0;

static uint8_t                         mPhyIrqDisableCnt = 1;
static uint8_t rf_phy_channel = 0;
static uint8_t  need_ack = 0;



/* Channel configurations for 2.4 */
static const phy_rf_channel_configuration_s phy_24ghz = {2405000000U, 5000000U, 250000U, 16U, M_OQPSK};

static const phy_device_channel_page_s phy_channel_pages[] = {
        { CHANNEL_PAGE_0, &phy_24ghz},
        { CHANNEL_PAGE_0, NULL}
};
/* Set the default power level to 0dBm */
#ifndef gPhyDefaultTxPowerLevel_d
#define gPhyDefaultTxPowerLevel_d     (22)
#endif
#define CORE_CLOCK_FREQ 47972352U
#define gPhyMaxTxPowerLevel_d         (32)
uint8_t gPhyChannelTxPowerLimits[] = { gPhyMaxTxPowerLevel_d,   /* 11 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 12 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 13 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 14 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 15 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 16 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 17 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 18 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 19 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 20 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 21 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 22 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 23 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 24 */ \
                                 gPhyMaxTxPowerLevel_d,   /* 25 */ \
                                 gPhyMaxTxPowerLevel_d };  /* 26 */



MBED_UNUSED static void rf_init(void);
MBED_UNUSED static int8_t rf_device_register(void);
MBED_UNUSED static void rf_device_unregister(void);
MBED_UNUSED static int8_t  rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
MBED_UNUSED static int8_t  rf_extension(phy_extension_type_e extension_type,uint8_t *data_ptr);
MBED_UNUSED static int8_t  rf_address_write(phy_address_type_e address_type,uint8_t *address_ptr);
MBED_UNUSED static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol );
MBED_UNUSED static void    rf_abort(void);
MBED_UNUSED static void    rf_promiscuous(uint8_t mode);
MBED_UNUSED static void rf_receive(void);
MBED_UNUSED static void rf_mac64_read(uint8_t *address);
MBED_UNUSED static void    rf_set_power_state(phyPwrMode_t newState);
MBED_UNUSED static void rf_handle_tx_end(uint8_t);


static void PHY_InterruptHandler(void);
static void PHY_InterruptThread(void);
static void handle_interrupt(void);

static void rf_if_lock(void)
{
    platform_enter_critical();
}

static void rf_if_unlock(void)
{
    platform_exit_critical();
}

static phyStatus_t PhyPlmeSetPwrLevelRequest
(
  uint8_t pwrStep
)
{
    phyStatus_t status = gPhySuccess_c;
#ifdef PHY_PARAMETERS_VALIDATION
    if( pwrStep > 32 )
    {
        status = gPhyInvalidParameter_c;
    }
    else
#endif /* PHY_PARAMETERS_VALIDATION */
    {
        /* Do not exceed the Tx power limit for the current channel */
        if( pwrStep > gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM0 - 11] )
        {
            pwrStep = gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM0 - 11];
        }
        if( pwrStep > 2 )
        {
            pwrStep = (pwrStep << 1) - 2;
        }
        
        ZLL->PA_PWR = pwrStep;
    }
    return status;
}

static uint8_t PhyPlmeGetPwrLevelRequest(void)
{
    uint8_t pwrStep = (uint8_t)ZLL->PA_PWR;

    if( pwrStep > 2 )
    {
        pwrStep = (pwrStep + 2) >> 1;
    }
    
    return pwrStep;
}

static phyStatus_t PhyPlmeSetCurrentChannelRequest
(
  uint8_t channel,
  uint8_t pan
)
{
    phyStatus_t status = gPhySuccess_c;
#ifdef PHY_PARAMETERS_VALIDATION
    if((channel < 11) || (channel > 26))
    {
        status = gPhyInvalidParameter_c;
    }
    else
#endif /* PHY_PARAMETERS_VALIDATION */
    {
        if( !pan )
        {
            ZLL->CHANNEL_NUM0 = channel;
        }
        else
        {
            ZLL->CHANNEL_NUM1 = channel;
        }
        
        /* Make sure the current Tx power doesn't exceed the Tx power limit for the new channel */
        if( PhyPlmeGetPwrLevelRequest() > gPhyChannelTxPowerLimits[channel - 11] )
        {
            PhyPlmeSetPwrLevelRequest(gPhyChannelTxPowerLimits[channel - 11]);
        }
    }
    
    return status;
}
static void UnprotectFromXcvrInterrupt(void)
{
    platform_enter_critical();
    
    if( mPhyIrqDisableCnt )
    {
        mPhyIrqDisableCnt--;
        
        if( mPhyIrqDisableCnt == 0 )
        {
            ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_TRCV_MSK_MASK;
        }
    }

    platform_exit_critical();
}
#ifndef gPhyIrqPriority_c
#define gPhyIrqPriority_c     (0x80)
#endif
#define gPhyIrqNo_d           (Radio_1_IRQn)
static void PHY_InstallIsr()
{
    NVIC_SetVector(gPhyIrqNo_d, (uint32_t)&PHY_InterruptHandler);
    NVIC_ClearPendingIRQ(gPhyIrqNo_d);
    NVIC_EnableIRQ(gPhyIrqNo_d);    
    /* set transceiver interrupt priority */
    //NVIC_SetPriority(gPhyIrqNo_d, gPhyIrqPriority_c >> (8 - __NVIC_PRIO_BITS));
    UnprotectFromXcvrInterrupt();    
}

static void rf_channel_set(uint8_t channel)
{
    rf_phy_channel = channel;
    PhyPlmeSetCurrentChannelRequest(channel, 0); /* 2405 MHz */
}

static void rf_init(void)
{
    uint32_t phyReg;
    XCVR_Init( ZIGBEE_MODE, DR_500KBPS );
    
    XCVR_SetXtalTrim(0x30);
    
    /* Enable 16 bit mode for TC2 - TC2 prime EN, disable all timers,
       enable AUTOACK, mask all interrupts */
    ZLL->PHY_CTRL = (gCcaCCA_MODE1_c << ZLL_PHY_CTRL_CCATYPE_SHIFT) |
                   ZLL_PHY_CTRL_TC2PRIME_EN_MASK    |
                   ZLL_PHY_CTRL_TSM_MSK_MASK        |
                   ZLL_PHY_CTRL_WAKE_MSK_MASK       |
                   ZLL_PHY_CTRL_CRC_MSK_MASK        |
                   ZLL_PHY_CTRL_PLL_UNLOCK_MSK_MASK |
                   ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK |
                   ZLL_PHY_CTRL_RX_WMRK_MSK_MASK    |
                   ZLL_PHY_CTRL_CCAMSK_MASK         |
                   ZLL_PHY_CTRL_RXMSK_MASK          |
                   ZLL_PHY_CTRL_TXMSK_MASK          |
                   ZLL_PHY_CTRL_SEQMSK_MASK         |
                   ZLL_PHY_CTRL_AUTOACK_MASK        |
                   ZLL_PHY_CTRL_TRCV_MSK_MASK;

    /* Clear all PP IRQ bits to avoid unexpected interrupts immediately after init
       disable all timer interrupts */
    ZLL->IRQSTS = ZLL->IRQSTS;

    /* Enable Source Addresing Match module */
    ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAP0_EN_MASK;

    /* Clear HW indirect queue */
    ZLL->SAM_TABLE |= ZLL_SAM_TABLE_INVALIDATE_ALL_MASK;

    /*  Frame Filtering
        FRM_VER[7:6] = b11. Accept FrameVersion 0 and 1 packets, reject all others */
    ZLL->RX_FRAME_FILTER &= ~ZLL_RX_FRAME_FILTER_FRM_VER_FILTER_MASK;
    ZLL->RX_FRAME_FILTER = ZLL_RX_FRAME_FILTER_FRM_VER_FILTER(3) |
                           ZLL_RX_FRAME_FILTER_CMD_FT_MASK  |
                           ZLL_RX_FRAME_FILTER_DATA_FT_MASK |
                           ZLL_RX_FRAME_FILTER_BEACON_FT_MASK;

    /* Set prescaller to obtain 1 symbol (16us) timebase */
    ZLL->TMR_PRESCALE = 0x05;

    /* Set CCA threshold to -75 dBm */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_CCA1_THRESH(0xB5);

    /* Set the default power level */
    PhyPlmeSetPwrLevelRequest(gPhyDefaultTxPowerLevel_d);

    /* Adjust ACK delay to fulfill the 802.15.4 turnaround requirements */
    ZLL->ACKDELAY &= ~ZLL_ACKDELAY_ACKDELAY_MASK;
    ZLL->ACKDELAY |= ZLL_ACKDELAY_ACKDELAY(-8);
    
    /* Adjust LQI compensation */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP(96);

    /* Enable the RxWatermark IRQ and FilterFail IRQ */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_RX_WMRK_MSK_MASK;
    /* Set Rx watermark level */
    ZLL->RX_WTR_MARK = 0;

    /*Read eui64*/
    rf_mac64_read(MAC_address);
    /*set default channel to 11*/
    rf_channel_set(12);
    /*Start receiver*/
    
    /* DSM settings */
    phyReg = (RSIM->RF_OSC_CTRL & RSIM_RF_OSC_CTRL_BB_XTAL_READY_COUNT_SEL_MASK) >> 
             RSIM_RF_OSC_CTRL_BB_XTAL_READY_COUNT_SEL_SHIFT;
    phyReg = (1024U << phyReg) / (CORE_CLOCK_FREQ / 32768) + 1;
    RSIM->DSM_OSC_OFFSET = phyReg;

    /* Install PHY ISR */
    PHY_InstallIsr();
    rf_receive();    
}

static void rf_mac64_read(uint8_t *address)
{
    uint64_t val = ZLL->MACLONGADDRS0_MSB;
    val <<= 32;
    val |= ZLL->MACLONGADDRS0_LSB;

    *((uint64_t*)address) = val;
}

static void rf_promiscuous(uint8_t state)
{
    if( state )
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_PROMISCUOUS_MASK;
        /* FRM_VER[11:8] = b1111. Any FrameVersion accepted */
        ZLL->RX_FRAME_FILTER |= (ZLL_RX_FRAME_FILTER_FRM_VER_FILTER_MASK |
                                 ZLL_RX_FRAME_FILTER_ACK_FT_MASK |
                                 ZLL_RX_FRAME_FILTER_NS_FT_MASK);
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_PROMISCUOUS_MASK;
        /* FRM_VER[11:8] = b0011. Accept FrameVersion 0 and 1 packets, reject all others */
        /* Beacon, Data and MAC command frame types accepted */
        ZLL->RX_FRAME_FILTER &= ~(ZLL_RX_FRAME_FILTER_FRM_VER_FILTER_MASK |
                                 ZLL_RX_FRAME_FILTER_ACK_FT_MASK  |
                                 ZLL_RX_FRAME_FILTER_NS_FT_MASK   |
                                 ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK);
        ZLL->RX_FRAME_FILTER |= ZLL_RX_FRAME_FILTER_FRM_VER_FILTER(3);
    }    
}

static void rf_receive(void)
{
    uint32_t irqSts;
    /* RX can start only from Idle state */
    if( mPhyState != gIdle_c )
    {
        return;
    }
    rf_set_power_state(gPhyPwrAutodoze_c);

    /* Ensure that no spurious interrupts are raised, but do not change TMR1 and TMR4 IRQ status */
    irqSts = ZLL->IRQSTS;
    irqSts &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    irqSts |= ZLL_IRQSTS_TMR3MSK_MASK;
    ZLL->IRQSTS = irqSts;

    /* Start the RX sequence */
    ZLL->PHY_CTRL |= gRX_c ;
    /* unmask SEQ interrupt */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SEQMSK_MASK;
}

static int8_t rf_device_register(void)
{

    rf_init();

    /*Set pointer to MAC address*/
    device_driver.PHY_MAC = MAC_address;
    device_driver.driver_description = (char*)"NXP_KWx";

    //Create setup Used Radio chips
    /*Type of RF PHY is SubGHz*/
    device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;

    device_driver.phy_channel_pages = phy_channel_pages;
    /*Maximum size of payload is 127*/
    device_driver.phy_MTU = 127;
    /*No header in PHY*/
    device_driver.phy_header_length = 0;
    /*No tail in PHY*/
    device_driver.phy_tail_length = 0;
    /*Set address write function*/
    device_driver.address_write = &rf_address_write;
    /*Set RF extension function*/
    device_driver.extension = &rf_extension;
    /*Set RF state control function*/
    device_driver.state_control = &rf_interface_state_control;
    /*Set transmit function*/
    device_driver.tx = &rf_start_cca;
    /*Upper layer callbacks init to NULL*/
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    /*Virtual upper data callback init to NULL*/
    device_driver.arm_net_virtual_rx_cb = NULL;
    device_driver.arm_net_virtual_tx_cb = NULL;

    /*Register device driver*/
    rf_radio_driver_id = arm_net_phy_register(&device_driver);

    return rf_radio_driver_id;
}

static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol )
{
    uint8_t* pPB = (uint8_t*)ZLL->PKT_BUFFER_TX;
    pPB[0] = data_length + 2; /* including 2 bytes of FCS */
    memcpy(&pPB[1], data_ptr, data_length);

    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCABFRTX_MASK;
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_CCATYPE_MASK;
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCATYPE(gCcaCCA_MODE1_c);    

    mac_tx_handle = tx_handle;
    need_ack = (*data_ptr & 0x20) == 0x20;

    /* Set XCVR power state in run mode */
    rf_set_power_state(gPhyPwrAutodoze_c);
    mPhyState = gCCA_c;

    uint8_t xcvseq;    
    if(need_ack)
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_RXACKRQD_MASK;
        xcvseq = gTR_c;
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_RXACKRQD_MASK;
        xcvseq = gTX_c;        
    }    
    uint32_t irqSts;
    /* Ensure that no spurious interrupts are raised(do not change TMR1 and TMR4 IRQ status) */
    irqSts = ZLL->IRQSTS;
    irqSts &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    irqSts |= ZLL_IRQSTS_TMR3MSK_MASK;
    ZLL->IRQSTS = irqSts;

    /* Start the TX / TRX / CCA sequence */
    ZLL->PHY_CTRL |= xcvseq;
    /* Unmask SEQ interrupt */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SEQMSK_MASK;    

    return 0;
}


static void rf_set_short_adr(uint8_t * short_address)
{
    uint16_t value;
    memcpy(&value, short_address, sizeof(value));
    ZLL->MACSHORTADDRS0 &= ~ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK;
    ZLL->MACSHORTADDRS0 |= ZLL_MACSHORTADDRS0_MACSHORTADDRS0(value);    
}

static void rf_set_pan_id(uint8_t *pan_id)
{
    uint16_t value;
    memcpy(&value, pan_id, sizeof(value));
    ZLL->MACSHORTADDRS0 &= ~ZLL_MACSHORTADDRS0_MACPANID0_MASK;
    ZLL->MACSHORTADDRS0 |= ZLL_MACSHORTADDRS0_MACPANID0(value);
}

static void rf_set_address(uint8_t *address)
{
    uint32_t addrLo;
    uint32_t addrHi;    
    memcpy(&addrLo, address, sizeof(addrLo));
    address += sizeof(addrLo);
    memcpy(&addrHi, address, sizeof(addrHi));

    ZLL->MACLONGADDRS0_LSB = addrLo;
    ZLL->MACLONGADDRS0_MSB = addrHi;    
} 

static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    int8_t ret_val = 0;
    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            break;
            /*Set 64-bit address*/
        case PHY_MAC_64BIT:
            rf_set_address(address_ptr);
            break;
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            rf_set_short_adr(address_ptr);
            break;
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            rf_set_pan_id(address_ptr);
            break;
    }
    return ret_val;
}

static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    assert(0);
    return 0;
}

#define mPhyDSM_GuardTime_d         (5) /* DSM_TIME ticks (32.768KHz) */
static uint32_t mPhyDSMDuration = 0xFFFFF0;
static void rf_set_power_state(phyPwrMode_t newState)
{
    /* Parameter validation */
    if( newState > gPhyPwrReset_c )
    {
        return;
    }
    /* Check if the new power state = old power state */
    else if( newState == mPhyPwrState )
    {
        return;
    }
    else
    {
        switch(newState)
        {
            case gPhyPwrIdle_c:
                /* Set XCVR in run mode if not allready */
                if(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_ZIG_DEEP_SLEEP_STATUS_MASK)
                {
                    RSIM->ZIG_WAKE = (RSIM->DSM_TIMER + mPhyDSM_GuardTime_d);
                    while(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_ZIG_DEEP_SLEEP_STATUS_MASK);
                }
                break;

            case gPhyPwrDSM_c:
                /* Set XCVR in low power mode if not allready */
                if(!(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_ZIG_DEEP_SLEEP_STATUS_MASK))
                {
                    uint32_t minTime = (RSIM->DSM_OSC_OFFSET > mPhyDSM_GuardTime_d) ? RSIM->DSM_OSC_OFFSET : mPhyDSM_GuardTime_d;

                    RSIM->ZIG_SLEEP = RSIM->DSM_TIMER + minTime;
                    RSIM->ZIG_WAKE = RSIM->DSM_TIMER + mPhyDSMDuration;
                    RSIM->DSM_CONTROL |= RSIM_DSM_CONTROL_ZIG_SYSCLK_REQUEST_EN_MASK |
                        RSIM_DSM_CONTROL_DSM_TIMER_EN_MASK | 
                        RSIM_DSM_CONTROL_ZIG_SYSCLK_INTERRUPT_EN_MASK;
                }
                break;

            default:
                /* do not change current state */
                newState = mPhyPwrState;
                break;
        }

        mPhyPwrState = newState;
    }
}

static void rf_off(void)
{
    /* Abort any ongoing sequences */
    rf_abort();
    /* Set XCVR in a low power state */
    rf_set_power_state(gPhyPwrHibernate_c);
}

static void rf_shutdown(void)
{
    /*Call RF OFF*/
    rf_off();
}

static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
    switch (new_state)
    {
        /*Reset PHY driver and set to idle*/
        case PHY_INTERFACE_RESET:
            break;
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
            rf_shutdown();
            break;
        /*Enable PHY Interface driver*/
        case PHY_INTERFACE_UP:
            rf_channel_set(rf_channel);
            rf_receive();
            break;
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
            rf_abort();
            rf_channel_set(rf_channel);
            break;
        case PHY_INTERFACE_SNIFFER_STATE:             /**< Enable Sniffer state */
            rf_promiscuous(1);
            rf_channel_set(rf_channel);
            rf_receive();
            break;
    }
    return ret_val;
}

static void rf_device_unregister(void)
{
    arm_net_phy_unregister(rf_radio_driver_id);
}

static void rf_abort(void)
{
    rf_if_lock();
    /* Mask SEQ interrupt */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_SEQMSK_MASK;
    /* Disable timer trigger (for scheduled XCVSEQ) */
    if( ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMRTRIGEN_MASK )
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_TMRTRIGEN_MASK;
        /* give the FSM enough time to start if it was triggered */
        while( (XCVR_MISC->XCVR_CTRL & XCVR_CTRL_XCVR_STATUS_TSM_COUNT_MASK) == 0) {}
    }

    /* If XCVR is not idle, abort current SEQ */
    if( ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK )
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_XCVSEQ_MASK;
        /* wait for Sequence Idle (if not already) */
        while( ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK ) {}
    }

    /* Stop timers */
    ZLL->PHY_CTRL &= ~(ZLL_PHY_CTRL_TMR2CMP_EN_MASK |
            ZLL_PHY_CTRL_TMR3CMP_EN_MASK |
            ZLL_PHY_CTRL_TC3TMOUT_MASK );
    /* clear all PP IRQ bits to avoid unexpected interrupts( do not change TMR1 and TMR4 IRQ status ) */
    ZLL->IRQSTS &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);

    rf_if_unlock();
}



/*
 * \brief Function is a RF interrupt vector. End of frame in RX and TX are handled here as well as CCA process interrupt.
 *
 * \param none
 *
 * \return none
 */
static void PHY_InterruptHandler(void)
{
    /* Disable and clear transceiver(IRQ_B) interrupt */
    //MCR20Drv_IRQ_Disable();
    //irq_thread.signal_set(1);
        handle_interrupt();
}

static void PHY_InterruptThread(void)
{
    for (;;) {
        osEvent event = irq_thread.signal_wait(0);
        if (event.status != osEventSignal) {
            continue;
        }
        handle_interrupt();
    }
}

static void PhyIsrSeqCleanup
(
  void
)
{
    uint32_t irqStatus;

    /* Set the PHY sequencer back to IDLE */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_XCVSEQ_MASK;
    /* Mask SEQ, RX, TX and CCA interrupts */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCAMSK_MASK |
                     ZLL_PHY_CTRL_RXMSK_MASK  |
                     ZLL_PHY_CTRL_TXMSK_MASK  |
                     ZLL_PHY_CTRL_SEQMSK_MASK;

    while( ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK ) {}

    irqStatus = ZLL->IRQSTS;
    /* Mask TMR3 interrupt */
    irqStatus |= ZLL_IRQSTS_TMR3MSK_MASK;
    /* Clear transceiver interrupts except TMRxIRQ */
    irqStatus &= ~( ZLL_IRQSTS_TMR1IRQ_MASK |
                    ZLL_IRQSTS_TMR2IRQ_MASK |
                    ZLL_IRQSTS_TMR3IRQ_MASK |
                    ZLL_IRQSTS_TMR4IRQ_MASK );
    ZLL->IRQSTS = irqStatus;
}

static void PhyIsrTimeoutCleanup
(
  void
)
{
    uint32_t irqStatus;

    /* Set the PHY sequencer back to IDLE and disable TMR3 comparator and timeout */
    ZLL->PHY_CTRL &= ~(ZLL_PHY_CTRL_TMR3CMP_EN_MASK | 
                       ZLL_PHY_CTRL_TC3TMOUT_MASK   | 
                       ZLL_PHY_CTRL_XCVSEQ_MASK);
    /* Mask SEQ, RX, TX and CCA interrupts */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCAMSK_MASK |
                     ZLL_PHY_CTRL_RXMSK_MASK  |
                     ZLL_PHY_CTRL_TXMSK_MASK  |
                     ZLL_PHY_CTRL_SEQMSK_MASK;

    while( ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK ) {}

    irqStatus = ZLL->IRQSTS;
    /* Mask TMR3 interrupt */
    irqStatus |= ZLL_IRQSTS_TMR3MSK_MASK;
    /* Clear transceiver interrupts except TMR1IRQ and TMR4IRQ. */
    irqStatus &= ~( ZLL_IRQSTS_TMR1IRQ_MASK |
                    ZLL_IRQSTS_TMR4IRQ_MASK );
    ZLL->IRQSTS = irqStatus;
}

static void handle_interrupt(void)
{
 /* RSIM Wake-up IRQ */
    if(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_ZIG_SYSCLK_REQ_INT_MASK)
    {
        RSIM->DSM_CONTROL = RSIM->DSM_CONTROL;
        return;
    }

    /* Read current XCVRSEQ and interrup status */
    uint32_t xcvseqCopy = ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK;
    uint32_t irqStatus     = ZLL->IRQSTS;
    /* Clear all xcvr interrupts */
    ZLL->IRQSTS = irqStatus;

    /* WAKE IRQ */
    if( irqStatus & ZLL_IRQSTS_WAKE_IRQ_MASK )
    {
        uint32_t timeAdjust = RSIM->ZIG_WAKE;
        /* Adjust the 802.15.4 EVENT_TMR */
        timeAdjust = (timeAdjust - RSIM->ZIG_SLEEP)/32768U * 1000000U; /* [us] */
        ZLL->EVENT_TMR = (timeAdjust << 4) | ZLL_EVENT_TMR_EVENT_TMR_ADD_MASK;
        //Radio_Phy_TimeRxTimeoutIndication(mPhyInstance);
    }

    /* Flter Fail IRQ */
    if( irqStatus & ZLL_IRQSTS_FILTERFAIL_IRQ_MASK )
    {
    }
    /* Rx Watermark IRQ */
    else
    {
        if( (!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_RX_WMRK_MSK_MASK)) && (irqStatus & ZLL_IRQSTS_RXWTRMRKIRQ_MASK) )
        {
            uint32_t length = (irqStatus & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK) >> ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT;
        }
    }

    /* Timer 1 Compare Match */
    if( (irqStatus & ZLL_IRQSTS_TMR1IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR1MSK_MASK)) )
    {
    }

    /* Sequencer interrupt, the autosequence has completed */
    if( (!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_SEQMSK_MASK)) && (irqStatus & ZLL_IRQSTS_SEQIRQ_MASK) )
    {
        /* PLL unlock, the autosequence has been aborted due to PLL unlock */
        if( irqStatus & ZLL_IRQSTS_PLL_UNLOCK_IRQ_MASK )
        {
            PhyIsrSeqCleanup();
        }
        /* TMR3 timeout, the autosequence has been aborted due to TMR3 timeout */
        else if( (irqStatus & ZLL_IRQSTS_TMR3IRQ_MASK) &&
            (!(irqStatus & ZLL_IRQSTS_RXIRQ_MASK)) &&
            (gTX_c != xcvseqCopy) )
        {
            PhyIsrTimeoutCleanup();
            //Radio_Phy_TimeRxTimeoutIndication(mPhyInstance);
        }
        else
        {
            PhyIsrSeqCleanup();
            switch(xcvseqCopy)
            {
            case gTX_c:
                if( (ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCABFRTX_MASK) && (irqStatus & ZLL_IRQSTS_CCA_MASK ) )
                {
                     device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 1, 1);
                }
                else
                {
                    rf_handle_tx_end(0);
                }
                break;
                
            case gTR_c:
                if( (ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCABFRTX_MASK) && (irqStatus & ZLL_IRQSTS_CCA_MASK ) )
                {
                    //Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, mPhyInstance);
                }
                else
                {
                    //Phy_GetRxParams();
                    rf_handle_tx_end((irqStatus & ZLL_IRQSTS_RX_FRM_PEND_MASK) > 0);

                }
                break;
                
            case gRX_c:
                /* Check SAA0 and SAA1 (source address absent) */
                if( irqStatus & ZLL_IRQSTS_PI_MASK )
                {
#if 0
                    if( PhyPpIsTxAckDataPending() )
                    {
                        phyLocal.flags |= gPhyFlagTxAckFP_c;
                    }
                    else
                    {
                        phyLocal.flags &= ~gPhyFlagTxAckFP_c;
                    }
                    
                    if( ZLL->SAM_MATCH & (ZLL_SAM_MATCH_SAA0_ADDR_ABSENT_MASK | ZLL_SAM_MATCH_SAA1_ADDR_ABSENT_MASK) )
                    {
                        mPhyForceFP = TRUE;
                    }
#endif
                }
                
                //Phy_GetRxParams();
                //Radio_Phy_PdDataIndication(mPhyInstance);
                break;
                
            case gCCA_c:
                if( gCcaED_c == ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCATYPE_MASK) >> ZLL_PHY_CTRL_CCATYPE_SHIFT) )
                {
                    //Radio_Phy_PlmeEdConfirm( (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK) >> ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT, mPhyInstance );
                }
                else /* CCA */
                {
                    if( irqStatus & ZLL_IRQSTS_CCA_MASK )
                    {
                        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 1, 1);
                    }
                    else
                    {
                        //rf_start_tx();
                        //Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, mPhyInstance);
                    }
                }
                break;
                
            case gCCCA_c:
                //Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, mPhyInstance);
                break;
                
            default:
                break;
            }
        }
    }
    /* Timers interrupt */
    else
    {
        /* Timer 2 Compare Match */
        if( (irqStatus & ZLL_IRQSTS_TMR2IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR2MSK_MASK)) )
        {
            if( gIdle_c != xcvseqCopy )
            {
                //Radio_Phy_TimeStartEventIndication(mPhyInstance);
            }
        }

        /* Timer 3 Compare Match */
        if( (irqStatus & ZLL_IRQSTS_TMR3IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR3MSK_MASK)) )
        {

            /* Ensure that we're not issuing TimeoutIndication while the Automated sequence is still in progress */
            /* TMR3 can expire during R-T turnaround for example, case in which the sequence is not interrupted */
            if( gIdle_c == xcvseqCopy )
            {
                //Radio_Phy_TimeRxTimeoutIndication(mPhyInstance);
            }
        }

        /* Timer 4 Compare Match */
        if( (irqStatus & ZLL_IRQSTS_TMR4IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR4MSK_MASK)) )
        {
            /* Disable TMR4 comparator */
            ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_TMR4CMP_EN_MASK;
            /* Mask and clear TMR4 interrupt (do not change other IRQ status) */
            irqStatus &= ~( ZLL_IRQSTS_TMR1MSK_MASK |
                            ZLL_IRQSTS_TMR2MSK_MASK |
                            ZLL_IRQSTS_TMR3MSK_MASK );
            irqStatus |= ZLL_IRQSTS_TMR4IRQ_MASK | ZLL_IRQSTS_TMR4MSK_MASK;
            ZLL->IRQSTS = irqStatus;
        }
    }
}

static void rf_handle_tx_end(uint8_t rx_frame_pending)
{
    rf_receive();

    if (!device_driver.phy_tx_done_cb) {
        return;
    }
    if( need_ack )
    {
        if( rx_frame_pending )
        {
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_DONE_PENDING, 1, 1);
        }
        else
        {
            // arm_net_phy_tx_done(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_SUCCESS, 1, 1);
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_DONE, 1, 1);
        }
    }
    else
    {
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_SUCCESS, 1, 1);
    }


}

NanostackRfPhyKw41z::NanostackRfPhyKw41z()
{
}

NanostackRfPhyKw41z::~NanostackRfPhyKw41z()
{
}

int8_t NanostackRfPhyKw41z::rf_register()
{

    rf_if_lock();

    if (rf != NULL) {
        rf_if_unlock();
        error("Multiple registrations of NanostackRfPhyMcr20a not supported");
        return -1;
    }

    irq_thread.start(mbed::callback(PHY_InterruptThread));

    int8_t radio_id = rf_device_register();
    if (radio_id < 0) {
        rf = NULL;
    }

    rf_if_unlock();
    return radio_id;
}

void NanostackRfPhyKw41z::rf_unregister()
{
    rf_if_lock();

    if (rf != this) {
        rf_if_unlock();
        return;
    }

    rf_device_unregister();
    rf = NULL;
    rf_if_unlock();
}


void NanostackRfPhyKw41z::set_mac_address(uint8_t* mac)
{
    rf_if_lock();

    if (NULL != rf) {
        error("NanostackRfPhyKw41z cannot change mac address when running");
        rf_if_unlock();
        return;
    }
    memcpy((void*)MAC_address, (void*)mac, sizeof(MAC_address));

    rf_if_unlock();
}

void NanostackRfPhyKw41z::get_mac_address(uint8_t *mac)
{
    rf_if_lock();

    memcpy((void*)mac, (void*)MAC_address, sizeof(MAC_address));

    rf_if_unlock();
}

