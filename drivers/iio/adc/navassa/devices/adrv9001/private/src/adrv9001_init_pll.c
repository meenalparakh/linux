/**
 * \file
 * \brief Contains ADRV9001 init PLL related private function implementations
 *
 * The PLL code was ported from Navassa firmware code and must be kept in sync with minimal change.
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_mcs.h"
#include "adrv9001_init.h"
#include "adi_common_macros.h"
#include "adrv9001_init_types.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_nvs_pll_mem_map.h"
#include "adrv9001_bf_nvs_regmap_core.h"

#define TEMP_SENSE_FREQ_TARGET      (32000000u)   /* 32MHz */
#define TEMP_SENSE_MAX_DIV          (63u)         /* Max allowable temperature sensor div */
#define TEMP_SENSE_MIN_DIV          (2u)          /* Min allowable temperature sensor div */
#define TEMP_SLOPE_DEF              (1360u)       /* Aux ADC Temp Slope */

#define ADC_TUNER_CAL_DELAY_USEC   (1u)          /*!< 1 usec */
#define ADC_TUNER_CAL_TIMEOUT_MSEC (2u)          /*!< 2 msec */

#define ONE_MEG                    (1000000U)
#define FIVE_HUNDRED_MEG         (500000000U)
#define ADC_CDFB_SHIFT                   (4U) /* To have accurate representatin of CDFB */
#define ADC_SC_SHIFT                     (8U) /* CCAL scale shift */
#define ADC_SG_SHIFT                    (11U) /* RCAL scale shift */
#define ADC_SCDFB_SHIFT                 (10U) /* CDFB scale shift, CDFB scale represents rouding error from CDFB quantization */
#define CCAL_NOMINAL_VAL (1U << ADC_SC_SHIFT) /* 1 in 8.8 format */
#define RCAL_NOMINAL_VAL (1U << ADC_SG_SHIFT) /* 1 in 5.11 format*/

#define BITM_ADC_TUNER_CAL_AT_STARTUP   (0X04U)    /* Calibration at startup */
#define BITM_ADC_TUNER_CCAL             (0X02U)    /* Capacitance calibration */
#define BITM_ADC_TUNER_RCAL             (0X01U)    /* Conductance calibration */

#define PLL_CAL_MAX_CNT_FAST                      (0x0011D1ul)
#define PLL_CAL_MAX_CNT_NORMAL                    (0x030D3Ful)

#ifdef LOOP_FILTER_TABLE_LOOKUP
/**
* \brief Find the Band Index
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
* \param vcoFreqHz - VCO freq of interest.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllFindBand(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings, uint64_t vcoFreq_Hz)
{
    uint32_t  index = 0;

    const uint32_t PLL_vcoFreqArrayKHz[ADRV9001_PLL_NUM_BANDS] =
    {
    12605000, 12245000, 11906000, 11588000, 11288000,
    11007000, 10742000, 10492000, 10258000, 10036000,
    9827800, 9631100, 9445300, 9269800, 9103600,
    8946300, 8797000, 8655300, 8520600, 8392300,
    8269900, 8153100, 8041400, 7934400, 7831800,
    7733200, 7638400, 7547100, 7459000, 7374000,
    7291900, 7212400, 7135500, 7061000, 6988700,
    6918600, 6850600, 6784600, 6720500, 6658200,
    6597800, 6539200, 6482300, 6427000, 6373400,
    6321400, 6270900, 6222000, 6174500, 6128400,
    6083600, 6040100, 5997700
    };

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    /* Find the band for the set freq */
    for (index = 0u; index < ADRV9001_PLL_NUM_BANDS-1; index++)
    {
        if ((vcoFreq_Hz < (((uint64_t)PLL_vcoFreqArrayKHz[index] * 1000ul))) &&
            (vcoFreq_Hz >= (((uint64_t)PLL_vcoFreqArrayKHz[index+1] * 1000ul))))
        {
            break;
        }
    }

    if (index >= ADRV9001_PLL_NUM_BANDS)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            index,
            "Error finding the band for the set freq");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    pllStateSettings->bandIndex = index;

    ADI_API_RETURN(device);
}
#endif

#define ENABLE_PHASE_SYNC            (0u)
#define PLL_TRADITIONAL_LOCK         (1)          /* Sets old lock algorithm, set to 0 for Fast Lock */

/* Use for generating loop filter table, FIXME: TDAO - Remove after bringup */
//#define ADRV9001_CALCULATE_LF 1

#ifdef ADRV9001_CALCULATE_LF
#include <math.h>
#define  DEF_PI     (3.1415926535897932f)

#define ICP_UNITS               (110.0e-6f)
#define ICP_MAX                 (64.0f * ICP_UNITS)
#define ICP_MIN                 (1.0f * ICP_UNITS)
#define CAP_UNITS               (64.0f - 1.0f)
#define RES_UINTS               (64.0f - 1.0f)
#define C1_UNITS                (30.3e-12f)
#define C1_MAX                  (C1_UNITS * (CAP_UNITS + 1.0f))
#define C1_MIN                  (C1_UNITS)
#define C2_UNITS                (1.78e-12f)
#define C2_MAX                  (C2_UNITS * (CAP_UNITS + 1.0f))
#define C2_MIN                  (C2_UNITS)
#define C3_UNITS                (0.717e-12f)
#define C3_MAX                  (C3_UNITS * (CAP_UNITS + 1.0f))
#define C3_MIN                  (C3_UNITS)
#define R1_UNITS                (31.14e3f)
#define R3_UNITS                (31.14e3f)

#define LF_INDUCTOR             (158.5e-12f)
#define LF_CPARA                (120.0e-15f)
#define LF_CVAR                 (40.0e-15f)

#define LF_CVAR_TC              (148.8e-15f)
#define LF_CCORE                (144.0e-15f)
#define LF_CPARA                (120.0e-15f)
#define LF_CVAR                 (40.0e-15f)
#define LF_CUINT_C              (8.0e-15f)
#define LF_CUINT_F              (1.1e-15f)
#define LF_CON_COFF_C           (11.8f)
#define LF_CON_COFF_F           (4.0f)
#define LF_DCUNIT_DV            (5.90e-15f)

/* Setting for LO vs LoopFilter settings */
#define LOOP_BW_LOFREQ_SETPOINT     (2000000000ul)

/**
 *******************************************************************************
 * Function: LF_CalcloopFilterCoef
 * \brief
 *
 * \details  Returns the loopfilter h/w settings for a specified PLL
 *
 * Parameters:
 * \param [in]   Structure pointer to ADRV9001 device data structure
 * \param [in]   Pointer to PLL state structure.
 * \param [out]  Loop filter H/W settings.
 *
 * \return
 *
 * Reference to other related functions
 * \sa
 *
 *   NOTE: This was a direct port from Matlab code provided by H/W
 *         the structure was not modified at this time to minimize
 *         introducing implementation issues. Optimization can be
 *         revisited latter.
 *
 *******************************************************************************
*/
/* Use for generating loop filter table*/
static void adrv9001_LfCalcloopFilterCoef(adi_adrv9001_Device_t *device, adrv9001_PllSynthParam_t *pPll, adrv9001_loopFilterResult_t *pResult)
{
    /* Setup constants */
    float nDiv;
    float fuInit, fuAct;
    float kVco;
    float fVco;
    float fzInit;
    //float fp1Init;
    //float fp2Init;
    float ratioFp2Fp1;
    float ratioFuFz;
    float ratioFp1Fu;
    float icpCalc2, icpAct;
    float r1Calc1, r1Calc2, r1Act;
    float /*r3Calc1, r3Calc2,*/ r3Act;
    float c1Calc1, c1Calc2, c1Calc3;
    float c2Calc3;
    float cTotal;
    float bandC, bandF;
    float Vout;
    float r1Min, r1Max;
    /* for FVCO=12800, band_c_calc=21 and Kv = 77.7 */
        /* I think these are the bit settings, if so, pass as an array */
    float bC1, bC2, bC3, bR3, bR1, bICP;

    /* float c2_calc1, c3_calc1, c2_calc2, c3_calc2*/
    float c3Calc3;

    /* Calculate NDiv, Kv and Vout. */
    Vout =.33f + (float)pPll->voutLvl / 15.0f;
    fVco = (float)pPll->vcoFreq_Hz;
    nDiv = fVco / (float)pPll->refClockFreq_Hz;
    fuInit = pPll->loopBW;
    ratioFp2Fp1 = 2.0f;
    bandF = 16.0f;

    {
        float const1, const2;

        if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
        {
            const1 = 255.0f;
            const2 = 31.0f;
        }
        else
        {
            const1 = 511.0f;
            const2 = 15.0f;
        }

        float  numer1, numer2, numer3, denom;
        numer1 =  1.0f / ((2.0f * DEF_PI * fVco) * (2.0f * DEF_PI * fVco)) / LF_INDUCTOR;
        numer2 = LF_CCORE + LF_CPARA + LF_CVAR + LF_CVAR_TC + (LF_CUINT_C * const1 / LF_CON_COFF_C);
        numer3 = LF_CUINT_F * (bandF + ((const2 - bandF) / LF_CON_COFF_F));

        denom =  LF_CUINT_C * (1.0f - (1.0f / LF_CON_COFF_C));
        bandC = (numer1 - (numer2 + numer3)) / denom;

        cTotal =  (LF_CVAR_TC + LF_CCORE + LF_CPARA + LF_CVAR)
                   + (LF_CUINT_C * (bandC + ((const1 - bandC) / LF_CON_COFF_C)))
                   + (LF_CUINT_F * (bandF + ((const2 - bandF) / LF_CON_COFF_F)));

        kVco = fVco * LF_DCUNIT_DV * ((1.0f + (float)pPll->vcoVaractor) / (2.0f * cTotal)) * (1.0f / Vout);
    }

    /* Calculate the Kv */
    kVco = fVco * 5.90e-15f * ((1.0f + (float)pPll->vcoVaractor) / (2.0f * cTotal)) * (1.0f / Vout);
    pPll->dbg_kv = kVco;
    /* Convert phase margin into Stablity  */
    {
        float k, sbf, phaseMarginRads;
        phaseMarginRads = (float)pPll->phaseMargin * DEF_PI / 180.0f;
        k = 1.0f / tanf(phaseMarginRads);
        sbf = (5.0f + sqrtf(25.0f + 32.0f * (k*k))) / (4.0f * k);
        ratioFuFz = sbf;
        ratioFp1Fu = sbf;
    }

    /* specify unit and max values (comments from MatLab)
    icp_unit = 100.0e-6f;
    icp_max = 6.4e-3f;
    icp_min = 100.0e-6f;
    cap_num_units = 64.0f - 1.0f;
    res_num_units = 64.0f - 1.0f;
    c1_unit = 28.656e-12f;
    c1_max = c1_unit * (cap_num_units + 1.0f);
    c1_min = c1_unit;
    c2_unit = 1.814e-12f;
    c2_max = c2_unit * (cap_num_units + 1.0f);
    c2_min = c2_unit;
    c3_unit = 0.7615e-12f;
    c3_max = c3_unit * (cap_num_units + 1.0f);
    c3_min = c3_unit;
    r1_unit = 31.14e3f;
    r3_unit = 31.14e3f;
    */

    /* step 1a. Calculate fz, fp1, fp2 */
    {
        fzInit = fuInit / ratioFuFz;
        //fp1Init = fuInit * ratioFp1Fu;
        //fp2Init = fp1Init * ratioFp2Fp1;

        /* step 1b. Calculate R1, C1, C2, R3, C3 */

        r1Calc1 = fuInit / (ICP_MAX / (2.0f * DEF_PI) * kVco) * nDiv;
        c1Calc1 = 1.0f / (2.0f * DEF_PI * fzInit * r1Calc1);
        //r3Calc1 = r1Calc1;

        /* step 2a. determine if C1 is realizable. set C1 to C1_max if C1>C1_max */
        if (c1Calc1 > C1_MAX)
        {
            c1Calc2 = C1_MAX;
        }
        else
        {
            c1Calc2 = c1Calc1;
        }
    }
    /* step 2b. Re-calculate R1, C2, R3, C3 based on new C1 from step2a */
    {
        //float c1C2ParaCalc2;

        r1Calc2 = 1.0f / (2.0f * DEF_PI * fzInit * c1Calc2);
        icpCalc2 = fuInit / (r1Calc2 * kVco) * nDiv * 2.0f * DEF_PI;
        if (icpCalc2 < ICP_MIN)
        {
            icpCalc2 = ICP_MIN;
        }
        //c1C2ParaCalc2 = 1.0f / (2.0f * DEF_PI * fp1Init * r1Calc2);
        //r3Calc2 = r1Calc2;
    }
    /* step 3a. round icp to nearest 100u */
    {
        float icpCalc3;

        bICP = roundf((icpCalc2 - ICP_UNITS) / ICP_UNITS);
        icpCalc3 = (bICP + 1.0f) * ICP_UNITS;
        if (icpCalc3 > (icpCalc2 * 1.15f))
        {
            bICP = bICP - 1.0f;
        }

        icpAct = (bICP + 1.0f)* ICP_UNITS;
    }
    /* step 3b. determine realizable R1 */
    {
        float bR1RoundUp, bR1RoundDn;
        float r1Calc3Up, r1Calc3Dn, r1Calc3Avg, r1Calc3;

        r1Calc3 = fuInit / (icpAct / (2.0f * DEF_PI) * kVco) * nDiv;

        r1Max = R1_UNITS;
        r1Min = R1_UNITS / (RES_UINTS + 1.0f);

        if (r1Calc3 > r1Max)
        {
            r1Calc3 = r1Max;
        }
        if (r1Calc3 < r1Min)
        {
            r1Calc3 = r1Min;
        }

        bR1RoundUp = RES_UINTS + 1.0f - floorf(R1_UNITS / r1Calc3);
        bR1RoundDn = RES_UINTS + 1.0f - ceilf(R1_UNITS / r1Calc3);

        r1Calc3Up = R1_UNITS / (RES_UINTS + 1.0f - bR1RoundUp);
        r1Calc3Dn = R1_UNITS / (RES_UINTS + 1.0f - bR1RoundDn);
        r1Calc3Avg = (r1Calc3Up + r1Calc3Dn) / 2.0f;
        if (r1Calc3 > r1Calc3Avg)
        {
            bR1 = bR1RoundUp;
        }
        else
        {
            bR1 = bR1RoundDn;
        }
        r1Act = R1_UNITS / (RES_UINTS  + 1.0f - bR1);
    }
    /* step 3c. calculate loop bw (fu) based on realizable icp, R1 (calc3) */
    {
        fuAct = icpAct / (2.0f * DEF_PI) * (r1Act * kVco) / nDiv;
    }

    /* step 3d. Calculate fz, fp1, fp2 based on fuAct */
    {
        float fzR1Act, fp1R1Act, fp2R1Act, c1c2ParaCalc3;

        fzR1Act = fuAct / ratioFuFz;
        fp1R1Act = fuAct * ratioFp1Fu;
        fp2R1Act = fp1R1Act * ratioFp2Fp1;


        /* step 3d. Re-calculate C1, C2, R3, C3 based on fuAct, r1Act */

        c1Calc3 = 1.0f / (2.0f * DEF_PI * fzR1Act * r1Act);
        c1c2ParaCalc3 = 1.0f / (2.0f * DEF_PI * fp1R1Act * r1Act);
        c2Calc3 = c1c2ParaCalc3 * c1Calc3 / (c1Calc3 - c1c2ParaCalc3);
        r3Act = r1Act;
        c3Calc3 = 1.0f / (2.0f *DEF_PI * fp2R1Act * r3Act);
    }

    /* step 3e. determine realizable C1, C2, R3, C3 */
    {

        if (c1Calc3 > C1_MAX)
        {
            c1Calc3 = C1_MAX;
        }
        if (c1Calc3 < C1_MIN)
        {
            c1Calc3 = C1_MIN;
        }

        if (c2Calc3 > C2_MAX)
        {
            c2Calc3 = C2_MAX;
        }

        if (c2Calc3 < C2_MIN)
        {
            c2Calc3 = C2_MIN;
        }
        if (c3Calc3 > C3_MAX)
        {
            c3Calc3 = C3_MAX;
        }

        if (c3Calc3 < C3_MIN)
        {
            c3Calc3 = C3_MIN;
        }

        bC1 = roundf(c1Calc3 / C1_MAX * (CAP_UNITS + 1.0f) - 1.0f);
        bC2 = roundf(c2Calc3 / C2_MAX * (CAP_UNITS + 1.0f) - 1.0f);
        bC3 = roundf(c3Calc3 / C3_MAX * (CAP_UNITS + 1.0f) - 1.0f);
        bR3 = bR1;

    }

    /* step 4a. Calculate actual fz, fp1, fp2 */
    pResult->effectiveLoopBW = (uint32_t)fuAct;
    {
#ifdef  LF_DEBUG

        c1_act = C1_UNITS*(1.0f + bC1);
        c2_act = C2_UNITS*(1.0f + bC2);
        c3_act = C3_UNITS*(1.0f + bC3);
        fz_act = 1.0f / (2.0f * DEF_PI * r1Act * c1_act);
        fp1_act = 1.0f / (2.0f * DEF_PI * r1Act / (1.0f / c1_act + 1.0f / c2_act));
        fp2_act = 1.0f / (2.0f * DEF_PI * r3Act * c3_act);
        pm_act = (atanf(fuAct / fz_act) - atanf(fuAct / fp1_act) - atanf(fuAct / fp2_act)) * 180.0f / DEF_PI;
#endif
    }


    /* step 4a. Calculate init and act PM */
    {
        /* Not sure if this is really necessary, possibly for debug */


    }
    /* Save results in output structure */
    pResult->C1 = (uint8_t)bC1;
    pResult->C2 = (uint8_t)bC2;
    pResult->C3 = (uint8_t)bC3;
    pResult->R1 = (uint8_t)bR1;
    pResult->R3 = (uint8_t)bR3;
    pResult->ICP = (uint8_t)bICP;
}
#endif

/**
* \brief     Returns the loopfilter h/w settings for a specified PLL
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllCalcloopFilterCoef(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    adrv9001_loopFilterResult_t result = { 0 };

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    /* If the PLL is Aux or sysclk, use the formula approach to find LF  */
    /* LF_CalcloopFilterCoef(pllStateSettings, &result); FIXME: TDAO get LF Coef */
#ifdef ADRV9001_CALCULATE_LF
    adrv9001_LfCalcloopFilterCoef(device, pllStateSettings, &result);
#else

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        /* Navassa A0: hardcoded result */
        result.C1  = 0x01u;
        result.C2  = 0x02u;
        result.C3  = 0x03u;
        result.R1  = 0x3Eu;
        result.R3  = 0x3Eu;
        result.ICP = 0x03u;
    }
    else
    {
        /* Navassa A0: hardcoded result */
        result.C1  = 0x00u;
        result.C2  = 0x01u;
        result.C3  = 0x01u;
        result.R1  = 0x3Eu;
        result.R3  = 0x3Eu;
        result.ICP = 0x08u;
    }

    result.effectiveLoopBW = 350000u;
#endif

    /* Save the results in selected PLLs shadow */
    pllStateSettings->loopFilter.C1 = result.C1;
    pllStateSettings->loopFilter.C2 = result.C2;
    pllStateSettings->loopFilter.C3 = result.C3;
    pllStateSettings->loopFilter.R1 = result.R1;
    pllStateSettings->loopFilter.R3 = result.R3;
    pllStateSettings->loopFilter.ICP = result.ICP;
    pllStateSettings->loopFilter.effectiveLoopBW = result.effectiveLoopBW;

    ADI_API_RETURN(device);
}

/**
* \brief     Calculates the int and frac words for the PLL h/w.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
* Notes:   Set Clock PLL int/fraction dividers (Double precsion math version)
*             N = vcoFreqMhz / scaledRefClkMhz;
*             integerWord=(uint16_t)floor(N);
*             fractionalWord=((N )-integerWord);
*             fractionalWord=fractionalWord * 8388593;
*             intFractionalWord=(uint32_t)ROUND(fractionalWord);
*/
static int32_t adrv9001_PllCalcIntFracWords(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    uint16_t      integerWord = 0;
    uint32_t      fractionalWord = 0;
    uint32_t      fractionalRemainder = 0;

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

#ifdef __KERNEL__
    integerWord = (uint16_t)div_u64_rem(pllStateSettings->vcoFreq_Hz, pllStateSettings->refClockFreq_Hz, &fractionalRemainder);
    /* +1 >> 1 is rounding (add .5) */
    fractionalWord = (uint32_t)(((div_u64(((uint64_t)fractionalRemainder * (ADRV9001_CLK_PLL_MODULUS << 1u)),
        pllStateSettings->refClockFreq_Hz)) + 1u) >> 1u);
#else
    integerWord         =  (uint16_t)(pllStateSettings->vcoFreq_Hz / pllStateSettings->refClockFreq_Hz);
    fractionalRemainder =  pllStateSettings->vcoFreq_Hz % pllStateSettings->refClockFreq_Hz;

    /* +1 >> 1 is rounding (add .5) */
    fractionalWord = (uint32_t) ( ( ( ( ( (uint64_t)fractionalRemainder  * (ADRV9001_CLK_PLL_MODULUS << 1u) )
                                  / (uint64_t)pllStateSettings->refClockFreq_Hz) ) + 1u) >> 1u);
#endif

    /* if fractionalWord rounded up and == PLL modulus, fix it */
    if (fractionalWord == ADRV9001_CLK_PLL_MODULUS)
    {
        fractionalWord = 0u;
        integerWord++;
    }

    /* Write the Int/Frac words to the PLL h/w Step 19.
       NOTE:  integerByte0 must BE LAST */
    pllStateSettings->fractionalByte0 = (uint8_t)(fractionalWord & 0xFFu);
    pllStateSettings->fractionalByte1 = (uint8_t)((fractionalWord >> 8u) & 0xFFu);
    pllStateSettings->fractionalByte2 = (uint8_t)((fractionalWord >> 16u) & 0x7Fu);
    pllStateSettings->integerByte1    = (uint8_t)((integerWord >> 8u) & 0x07u);
    pllStateSettings->integerByte0    = (uint8_t)(integerWord & 0xFFu);

    ADI_API_RETURN(device);
}

#if 0
static uint32_t adrv9001_utils_floorLog2U32(uint32_t x)
{
    uint32_t i = 0;
    uint32_t mask = 0x80000000;
    uint32_t y = 0;

    /* find most significant '1' bit */
    for (i = 0u; i < 31u; i++)
    {
        if (mask & x)
        {
            y = 31u - i;
            break;
        }
        mask >>= 1;
    }

    return y;
}

static uint32_t adrv9001_utils_ceilLog2U32(uint32_t x)
{
    uint32_t mask = 0;
    uint32_t y = 0;

    y = adrv9001_utils_floorLog2U32(x);
    mask = (1u << y) - 1u;
    if (mask & x)
    {
        y++;
    }

    return y;
}
#endif

/**
* \brief     Calculate the temperature coef for the selected PLL
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllUpdateTcCoef(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    uint64_t vcoMhz = 0;
    uint32_t index = 0;
    uint32_t tabSize = 0;
    const adrv9001_TempCoef_t *pTempComp;
    const adrv9001_TempCoef_t  PLL_TempCompRfPLL[3] = { /* Values are for RF 1/2 PLLs */
                                                        /*   f1,     f2,  REF    TCF   VCOVARTC  */
                                                       { 1u, 8669u, 7u, 7u, 31u },
                                                       { 8670u, 10869u, 6u, 5u, 31u },
                                                       { 10870u, 19000u, 5u, 3u, 20u } };

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

#ifdef __KERNEL__
    vcoMhz = div_u64(pllStateSettings->vcoFreq_Hz, 1000000u);
#else
    vcoMhz = pllStateSettings->vcoFreq_Hz / 1000000u;
#endif

    pTempComp = PLL_TempCompRfPLL;
    tabSize =  sizeof(PLL_TempCompRfPLL) / sizeof(adrv9001_TempCoef_t);

    /* Scan the table for a match */
    for (index = 0u; index != tabSize; index++)
    {
        if ((vcoMhz >= pTempComp[index].f1_MHz) && (vcoMhz <= pTempComp[index].f2_MHz))
        {
            break;
        }
    }
    /* Update the main PLL table */
    pllStateSettings->biasRef = pTempComp[index].biasRef;
    pllStateSettings->biasTcf = pTempComp[index].biasTcf;
    pllStateSettings->vcoVarTc = pTempComp[index].vcoVarTc;


    /* Navassa A0: hardcoded result */
    pllStateSettings->biasRef = 5u;
    pllStateSettings->biasTcf = 6u;

    ADI_API_RETURN(device);
}

/**
* \brief     Determine the ref clock setting for a PLL.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \return      Value for ref clock
*/
static uint32_t adrv9001_PllCalculatePllRefClock(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    uint8_t refDiv;
    uint8_t k1;
    uint32_t refClk = 0u;

    k1 = device->devStateInfo.clkDivideRatios.devClkDivideRatio;
    refDiv = device->devStateInfo.clkDivideRatios.refClkDivideRatio;
    pllStateSettings->refClkDiv = 1u << refDiv;
    pllStateSettings->k1ClkDiv  = 1u << k1;

    refClk = KILO_TO_BASE_UNIT(init->clocks.deviceClock_kHz) /
        pllStateSettings->refClkDiv / pllStateSettings->k1ClkDiv;

    return refClk;
}

/**
* \brief     Calculate the PLL parameters and populate into the selected
*              PLL config array (synth_param)
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllComputeSynthParameters(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t tableVal = 0;

    /* VCO out level vs VCO varactor setting */

    const uint8_t PllVcoVarNormal[] = { 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u }; /* For Normal Mode */
    const uint8_t PllVcoVarFfh[] = { 2u, 2u, 3u, 3u, 4u, 4u, 5u, 5u, 6u, 6u, 7u }; /* For FFH Mode */

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    /* Calculate the ref clock and save in shadow */
    pllStateSettings->refClockFreq_Hz = adrv9001_PllCalculatePllRefClock(device, init, pllStateSettings);

#ifdef LOOP_FILTER_TABLE_LOOKUP
    /* Find which band we are operating in */
    recoveryAction = adrv9001_PllFindBand(device, init, pllStateSettings, pllStateSettings->vcoFreq_Hz);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error Find which band we are operating in ");
    ADI_ERROR_RETURN(device->common.error.newAction);
#endif

    /* Calculate VCO Varactor based on Output level and mode (fast/slow).
       This Must be done before loop filter calcs since this will affect Kv
       Might as well set up param for Fast/Norm mode too */

    /* find Varactor based on Mode */
    if (pllStateSettings->mode == PLL_FAST_MODE)
    {
        /* Set freq_cal_max_cnt for Fast Mode */
        pllStateSettings->maxCnt = PLL_CAL_MAX_CNT_FAST;

        tableVal = PllVcoVarFfh[pllStateSettings->voutLvl];
    }
    else
    {
        /* Set freq_cal_max_cnt for Normal Mode */
        pllStateSettings->maxCnt = PLL_CAL_MAX_CNT_NORMAL;

        tableVal = PllVcoVarNormal[pllStateSettings->voutLvl];
    }

    /* Set Varactor setting. Must be done before calc loop filter coef */
    pllStateSettings->vcoVaractor = tableVal;

    /* Find the Loopfilter coef */
    recoveryAction = adrv9001_PllCalcloopFilterCoef(device, init, pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error in finding the Loopfilter coef ");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Calculate the fractional words */
    recoveryAction = adrv9001_PllCalcIntFracWords(device, init, pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error in calculating the fractional words ");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Set the VCO LUT as function of VCO Freq step 18 */
    recoveryAction = adrv9001_PllUpdateTcCoef(device, init, pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error in setting the VCO LUT as function of VCO Freq step 18 ");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* All the calculations are done so zero out the pll
       freq to inidcate that we are re-cal'ing */
    pllStateSettings->pllFreq_Hz = 0ul;

    ADI_API_RETURN(device);
}


/**
* \brief     Writes (using common spi writes), the pre-calculated PLL values
*              for the desired PLL.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllWriteSynthParameters(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t temp = 0;
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    static const uint8_t VCO_OUT_LVL_MASK            = 0x0F;
    static const uint8_t VCO_OUT_LVL_RESET           = 0x3A;
    static const uint8_t VCO_VAR_MASK                = 0x07;
    static const uint8_t VCO_VAR_RESET               = 0x01;
    static const uint8_t LF_RC_MASK                  = 0x3F;
    static const uint8_t CP_I_MASK                   = 0x3F;

    static const uint8_t DIVIDER_FRAC_BYTE2_MASK     = 0x7F;
    static const uint8_t DIVIDER_INT_BYTE1_MASK      = 0x07;
    static const uint8_t DIVIDER_INT_BYTE1_RESET     = 0x30;
    static const uint8_t VCO_BIAS_REF_MASK           = 0x07;
    static const uint8_t VCO_BIAS_TCF_MASK           = 0x70;
    static const uint8_t VCO_BIAS_TCF_SHIFT          = 4;

    static const uint8_t VCO_VAR_TC_MASK             = 0x1F;
    static const uint8_t VCO_TC_IDAC_SCALE_LSB_RESET = 0xC0;

    static const uint8_t LOCKDET_MODE_RESET          = 0x02;
    static const uint8_t LOCKDET_CNT_RESET           = 0x0C;
    static const uint8_t CLK_PLL_VCO_TC_BYTE3        = 0x80;

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    /* Update h/w reg if no errors */
    if (recoveryAction == ADI_COMMON_ACT_NO_ACTION)
    {
        /* Set VCO output level */
        temp = (VCO_OUT_LVL_RESET & ~VCO_OUT_LVL_MASK) |
                (pllStateSettings->voutLvl & VCO_OUT_LVL_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE2",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE2_OFFSET,
            temp);

        /* Set Varactor setting */
        temp = (VCO_VAR_RESET & ~VCO_VAR_MASK) |
                (pllStateSettings->vcoVaractor & VCO_VAR_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE1_OFFSET,
            temp);

        /* Write the Loopfilter words to the PLL h/w */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LF_C2",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LF_C2_OFFSET,
            pllStateSettings->loopFilter.C2 & LF_RC_MASK);

        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LF_C1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LF_C1_OFFSET,
            (pllStateSettings->loopFilter.C1 & LF_RC_MASK));

        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LF_R1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LF_R1_OFFSET,
            (pllStateSettings->loopFilter.R1 & LF_RC_MASK));

        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LF_C3",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LF_C3_OFFSET,
            (pllStateSettings->loopFilter.C3 & LF_RC_MASK));

        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LF_R3",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LF_R3_OFFSET,
            (pllStateSettings->loopFilter.R3 & LF_RC_MASK));

        /* Write the Chargepump words to the PLL h/w */
        /* WRITE_PMM_CP_I(base, pllStateSettings[pll].loopFilter.ICP); */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE3",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE3_OFFSET,
            (pllStateSettings->loopFilter.ICP & CP_I_MASK));

        /* Write in VCO LUT values step 20 */
        /* WRITE_PMM_VCO_BIAS_REF(base, pllStateSettings[pll].biasRef); */
        /* WRITE_PMM_VCO_BIAS_TCF(base, pllStateSettings[pll].biasTcf); */
        temp = pllStateSettings->biasRef & VCO_BIAS_REF_MASK;
        temp |= ((pllStateSettings->biasTcf << VCO_BIAS_TCF_SHIFT) & VCO_BIAS_TCF_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE9",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE9_OFFSET,
            temp);

        /* WRITE_PMM_VCO_VAR_TC(base, pllStateSettings[pll].vcoVarTc); */
        temp = VCO_TC_IDAC_SCALE_LSB_RESET;
        temp |= (pllStateSettings->vcoVarTc & VCO_VAR_TC_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_TC_BYTE1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_TC_BYTE1_OFFSET,
            temp);

        temp = CLK_PLL_VCO_TC_BYTE3;
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_VCO_TC_BYTE3",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_TC_BYTE3_OFFSET,
                              temp);

        /* Write the Int/Frac words to the PLL h/w Step 19.
           NOTE:  integerByte0 must BE LAST */
        /* WRITE_PMM_PLL_BASE_REGS_DIVIDER_FRAC_BYTE0_SDM_FRAC(base, pllStateSettings[pll].fractionalByte0); */
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_DIVIDER_FRAC_BYTE0",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_DIVIDER_FRAC_BYTE0_OFFSET,
                              pllStateSettings->fractionalByte0);

        /* WRITE_PMM_PLL_BASE_REGS_DIVIDER_FRAC_BYTE1_SDM_FRAC(base, pllStateSettings[pll].fractionalByte1); */
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_DIVIDER_FRAC_BYTE1",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_DIVIDER_FRAC_BYTE1_OFFSET,
                              pllStateSettings->fractionalByte1);

        /* WRITE_PMM_PLL_BASE_REGS_DIVIDER_FRAC_BYTE2_SDM_FRAC(base, pllStateSettings[pll].fractionalByte2); */
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_DIVIDER_FRAC_BYTE2",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_DIVIDER_FRAC_BYTE2_OFFSET,
                              (pllStateSettings->fractionalByte2 & DIVIDER_FRAC_BYTE2_MASK));

        /* WRITE_PMM_PLL_BASE_REGS_DIVIDER_INT_BYTE1_SDM_INT(base, pllStateSettings[pll].integerByte1); */
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_DIVIDER_INT_BYTE1",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_DIVIDER_INT_BYTE1_OFFSET,
                              (DIVIDER_INT_BYTE1_RESET | (pllStateSettings->integerByte1 & DIVIDER_INT_BYTE1_MASK)));

        /* WRITE_PMM_PLL_BASE_REGS_DIVIDER_INT_BYTE0_SDM_INT(base, pllStateSettings[pll].integerByte0); */
        ADRV9001_SPIWRITEBYTE(device,
                              "CLK_PLL_DIVIDER_INT_BYTE0",
                              baseAddr + ADRV9001_ADDR_CLK_PLL_DIVIDER_INT_BYTE0_OFFSET,
                              pllStateSettings->integerByte0);

#if 0
        /* WRITE_PMM_VCOCAL_MAXCNTBAND_EN(base, pllStateSettings[pll].vcoCalMaxCntBandEn); */
        /* WRITE_PMM_ENDVCOCAL_MAXCNT_EN(base, pllStateSettings[pll].endVcoCalMaxCntEn); */
        temp = (pllStateSettings->vcoCalMaxCntBandEn << VCOCAL_MAXCNTBAND_EN_SHIFT) & VCOCAL_MAXCNTBAND_EN_MASK;
        temp |= ((pllStateSettings->endVcoCalMaxCntEn << ENDVCOCAL_MAXCNT_EN_SHIFT) & ENDVCOCAL_MAXCNT_EN_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES",
            baseAddr + ADRV9001_ADDR_CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET,
            temp);
#endif

#if (PLL_TRADITIONAL_LOCK == 1)
        /* #warning "TODO: TPGFW-960 Check with Steve, this was done in sim code: Restore original setting after use if traditional lock detect is used"   */
        /* WRITE_PMM_LOCKDET_CNT(base, 0u); */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LOCKDET_CONFIG",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LOCKDET_CONFIG_OFFSET,
            (~LOCKDET_CNT_RESET & LOCKDET_MODE_RESET));
#endif
    }

    ADI_API_RETURN(device);
}

/**
* \brief     Used for SysClock during boot. Wait for temperature is valid.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adrv9001_PllWaitForTemp(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    uint32_t    timeOut = 0u;
    uint8_t     tempDone = 0u;
    uint8_t     bfValue = 0u;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;

    static const uint8_t TDEGC_DONE_MASK  = 0x80;
    static const uint8_t TDEGC_DONE_SHIFT = 7;

    /* We initiated a temp measurement cycle at boot, need to wait until its
       completed, or timeout */
    for (timeOut = 0u; timeOut != ADI_ADRV9001_TEMPERATURE_WAIT_US; timeOut++)
    {
        /* tempDone = READ_CORE_TDEGC_DONE(INST_SPI_CORE); */
        ADRV9001_SPIREADBYTE(device, "CORE_TDEGC_DONE",
            ADRV9001_ADDR_CORE_TDEGC_DONE,
            &bfValue);
        tempDone = (bfValue & TDEGC_DONE_MASK) >> TDEGC_DONE_SHIFT;

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            tempDone = 1;
        }
#endif
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
            tempDone = 1;
#endif
        if (tempDone == 1u)
        {
            break;
        }

        /* GPT_DelayUs(1u); */
        halError = adi_common_hal_Wait_us(&device->common, 1);

        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (timeOut == ADI_ADRV9001_TEMPERATURE_WAIT_US)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer timeout in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

/**
* \brief     Read the thermistor h/w and save in local storage.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllTempReadThermistor(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    uint8_t bfValue = 0;

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    /* pllStateSettings->temperature = (uint16_t)((READ_CORE_TEMPERATURE_SENSOR_TEMP_SENSE_RESULTS_LSB_TDEGC_READBACK(INST_SPI_CORE) & 0xFFu)); */
    /* PLLTEMP_temperature |= (((uint16_t)READ_CORE_TEMPERATURE_SENSOR_TEMP_SENSE_RESULTS_MSB_TDEGC_READBACK(INST_SPI_CORE)) << 0x08u); */
    ADRV9001_SPIREADBYTE(device, "CORE_LSB_TDEGC_READBACK",
        ADRV9001_ADDR_CORE_LSB_TDEGC_READBACK, &bfValue);

    pllStateSettings->temperature = (bfValue & 0xFF);

    ADRV9001_SPIREADBYTE(device, "CORE_MSB_TDEGC_READBACK",
        ADRV9001_ADDR_CORE_MSB_TDEGC_READBACK, &bfValue);

    pllStateSettings->temperature |= (((uint16_t)bfValue << 8) & 0x0F00);

    ADI_API_RETURN(device);
}

#if 0
static int32_t adrv9001_PllManualFineCoarseVcoCal(adi_adrv9001_Device_t *device, uint32_t base, adrv9001_PllVcoCalType_e calType, adrv9001_PllSynthParam_t *pllStateSettings)
{
    uint8_t temp = 0;
    static const uint8_t VCO_F_COARSE_BAND_EN_SET    = 0x02;
    static const uint8_t VCO_F_FINE_BAND_EN_SET      = 0x20;
    static const uint8_t VCO_F_FINE_BAND_RESET       = 0x10;
    static const uint8_t VCO_F_FINE_BAND_MASK        = 0x1F;

    static const uint8_t VCO_CAL_ALC_WAIT_MASK       = 0x70;
    static const uint8_t VCO_CAL_ALC_WAIT_SHIFT      = 4;
    static const uint8_t VCO_FINE_CAL_EN_MASK        = 0x08;
    static const uint8_t VCO_COARSE_CAL_EN_MASK      = 0x80;
    static const uint8_t VCO_CAL_ALC_WAIT_RESET      = 0x88;

    /* Setup for Coarse or Fine Cal */
    if (calType == PLL_COARSE_FINE_CAL)
    {
        /* WRITE_PMM_VCO_F_COARSE_BAND_EN(base, 1u); */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_F_COARSE_BAND_EN",
            base + ADRV9001_ADDR_CLK_PLL_VCO_F_COARSE_BAND_EN_OFFSET,
            VCO_F_COARSE_BAND_EN_SET);
        /* WRITE_PMM_VCO_F_FINE_BAND_EN(base, 0u); */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_F_FINE_BAND_EN",
            base + ADRV9001_ADDR_CLK_PLL_VCO_F_FINE_BAND_EN_OFFSET,
            (VCO_F_FINE_BAND_RESET & ~VCO_F_FINE_BAND_EN_SET));

        /* WRITE_PMM_VCO_COARSE_CAL_EN(base, 0u); */
        /* WRITE_PMM_VCO_FINE_CAL_EN(base, 1u); */
        temp = VCO_CAL_ALC_WAIT_RESET;
        temp &= ~VCO_COARSE_CAL_EN_MASK;
        temp |= VCO_FINE_CAL_EN_MASK;
        temp |= ((pllStateSettings->alcWait << VCO_CAL_ALC_WAIT_SHIFT) & VCO_CAL_ALC_WAIT_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_CAL_ALC_WAIT",
            base + ADRV9001_ADDR_CLK_PLL_VCO_CAL_ALC_WAIT_OFFSET,
            temp);

    }
    else
    {
        /* WRITE_PMM_VCO_F_COARSE_BAND_EN(base, 0u); */
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_F_COARSE_BAND_EN",
            base + ADRV9001_ADDR_CLK_PLL_VCO_F_COARSE_BAND_EN_OFFSET,
            (0 & ~VCO_F_COARSE_BAND_EN_SET));
        /* WRITE_PMM_VCO_COARSE_CAL_EN(base, ENABLE); */
        /* WRITE_PMM_VCO_FINE_CAL_EN(base, DISABLE); */
        temp = VCO_CAL_ALC_WAIT_RESET;
        temp &= ~VCO_FINE_CAL_EN_MASK;
        temp |= VCO_COARSE_CAL_EN_MASK;
        temp |= ((pllStateSettings->alcWait << VCO_CAL_ALC_WAIT_SHIFT) & VCO_CAL_ALC_WAIT_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_CAL_ALC_WAIT",
            base + ADRV9001_ADDR_CLK_PLL_VCO_CAL_ALC_WAIT_OFFSET,
            temp);

        /* WRITE_PMM_VCO_F_FINE_BAND_EN(base, ENABLE); */
        /* WRITE_PMM_VCO_F_FINE_BAND(base, 8u); */
        temp = VCO_F_FINE_BAND_EN_SET;
        temp |= (8u & VCO_F_FINE_BAND_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_F_FINE_BAND_EN",
            base + ADRV9001_ADDR_CLK_PLL_VCO_F_FINE_BAND_EN_OFFSET,
            temp);
    }

    ADI_API_RETURN(device);
}
#endif

/**
* \brief     Runs the VCO cal, Coarse and Fine.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param base - base address of the PLL
* \param type - cal type 0=fine, 1=Coarse
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllRunVCOCal(adi_adrv9001_Device_t *device, uint32_t base, adrv9001_PllVcoCalType_e type, adrv9001_PllSynthParam_t *pllStateSettings)
{
#if CHECK_VCO_CAL > 0
    uint8_t  calBusy = 0;
    uint8_t  bfValue = 0;
    uint32_t timeOut = 0u;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;

    static const uint8_t VCO_CAL_BUSY_MASK       = 0x02;
    static const uint8_t VCO_CAL_BUSY_SHIFT      = 1;
#endif
    static const uint8_t VCO_CAL_INIT_BYTE_RESET = 0x90;
    static const uint8_t VCO_CAL_INIT_SET        = 0x01;

#if 0   /* Navassa: moved to PLL_InitStaticRegisters() */
    //static const uint8_t VCO_F_COARSE_BAND_EN_SET    = 0x02;
    //static const uint8_t VCO_F_FINE_BAND_EN_SET      = 0x20;
    //static const uint8_t VCO_F_FINE_BAND_RESET       = 0x10;
    static const uint8_t VCO_F_FINE_BAND_MASK        = 0x1F;
    static const uint8_t VCO_F_FINE_BAND_VALUE       = 16;

    static const uint8_t VCO_CAL_ALC_WAIT_MASK       = 0x70;
    static const uint8_t VCO_CAL_ALC_WAIT_SHIFT      = 4;
    static const uint8_t VCO_FINE_CAL_EN_MASK        = 0x08;
    static const uint8_t VCO_COARSE_CAL_EN_MASK      = 0x80;
    static const uint8_t VCO_CAL_ALC_WAIT_RESET      = 0x88;


    static const uint8_t VCO_CAL_INIT_DEL_MASK       = 0xC0;
    static const uint8_t VCO_CAL_INIT_DEL_SHIFT      = 6;
    static const uint8_t VCO_CAL_ALC_INIT_WAIT_MASK  = 0x30;
    static const uint8_t VCO_CAL_ALC_INIT_WAIT_SHIFT = 4;
    static const uint8_t VCO_CAL_ALC_CLK_DIV_MASK    = 0x0C;
    static const uint8_t VCO_CAL_ALC_CLK_DIV_SHIFT   = 2;
    static const uint8_t VCO_CAL_ALC_STEP_RESET      = 0x03;

    static const uint8_t FORCE_VCO_INIT_ALC_VALUE    = 0x80;
    static const uint8_t VCOCAL_MAXCNTBAND_EN_MASK   = 0x02;
    static const uint8_t VCOCAL_MAXCNTBAND_EN_SHIFT  = 1;
    static const uint8_t ENDVCOCAL_MAXCNT_EN_MASK    = 0x04;
    static const uint8_t ENDVCOCAL_MAXCNT_EN_SHIFT   = 2;

    static const uint8_t VCO_2MA_PMOS_EN_SHIFT       = 2;
    static const uint8_t VCO_2MA_PMOS_EN_MASK        = 0x40;

    static const uint8_t Q_FREQ_CAL_THRESHOLD_VALUE  = 0x03;
    static const uint8_t Q_FREQ_CAL_EN_MASK          = 0x80;
    static const uint8_t Q_FREQ_CAL_EN_SHIFT         = 7;
#endif

    /* For A0 silicon we need to keep PLL_ManualFineCoarseVcoCal code IN to
   reliably run the VCO cal. For B0, this is no longer needed and the
   #else code is needed */

#ifdef A0_WORK_AROUNDS
    PLL_ManualFineCoarseVcoCal(base, type);
    /* Fix for bug in Si */
    recoveryAction = adrv9001_PllManualFineCoarseVcoCal(device, base, type, pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        type,
        "Error calling PllManualFineCoarseVcoCal ");
    ADI_ERROR_RETURN(device->common.error.newAction);
#else
#if 0   /* Navassa: moved to PLL_InitStaticRegisters() */
    /* Setup the PLL for the deisred cal type */
    if (type == PLL_COARSE_FINE_CAL)
    {
        /* vco_fine_cal_en=1
           vco_f_fine_band<4:0> = 'd8
           vco_cal_init_del<1:0>=3
           vco_cal_alc_init_wait<1:0>=3
           vco_cal_alc_clk_div<1:0>=3
           vco_cal_alc_wait<2:0>=7
           force_vco_init_alc_value=1
           vco_bias_dac_lvl_shift_2ma_pmos_en=0
           vco_var<2:0>=3 - already set in PLL_ComputeSynthParameters
           freq_cal_max_cnt<23:0> =  - already set in PLL_ComputeSynthParameters
        */
        //WRITE_CLK_PLL_VCO_F_FINE_BAND(base, 16u);
        bfValue = VCO_F_FINE_BAND_VALUE & VCO_F_FINE_BAND_MASK;
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_F_FINE_BAND",
            base + ADRV9001_ADDR_CLK_PLL_VCO_F_FINE_BAND_OFFSET,
            bfValue);

        //WRITE_CLK_PLL_VCO_COARSE_CAL_EN(base, 1u);
        //WRITE_CLK_PLL_VCO_FINE_CAL_EN(base, 1u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_WAIT(base, 7u);
        bfValue = VCO_CAL_ALC_WAIT_RESET;
        bfValue |= VCO_COARSE_CAL_EN_MASK;
        bfValue |= VCO_FINE_CAL_EN_MASK;
        bfValue |= ((7u << VCO_CAL_ALC_WAIT_SHIFT) & VCO_CAL_ALC_WAIT_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0",
            base + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0_OFFSET,
            bfValue);

        //WRITE_CLK_PLL_VCO_CAL_INIT_DEL(base, 3u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_INIT_WAIT(base, 3u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_CLK_DIV(base, 3u);
        bfValue = VCO_CAL_ALC_STEP_RESET;
        bfValue |= ((3u << VCO_CAL_INIT_DEL_SHIFT) & VCO_CAL_INIT_DEL_MASK);
        bfValue |= ((3u << VCO_CAL_ALC_INIT_WAIT_SHIFT) & VCO_CAL_ALC_INIT_WAIT_MASK);
        bfValue |= ((3u << VCO_CAL_ALC_CLK_DIV_SHIFT) & VCO_CAL_ALC_CLK_DIV_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1",
            base + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1_OFFSET,
            bfValue);

        //WRITE_CLK_PLL_FORCE_VCO_INIT_ALC_VALUE(base, 1u);
        bfValue = FORCE_VCO_INIT_ALC_VALUE;
        bfValue |= (pllStateSettings->vcoCalMaxCntBandEn << VCOCAL_MAXCNTBAND_EN_SHIFT) & VCOCAL_MAXCNTBAND_EN_MASK;
        bfValue |= ((pllStateSettings->endVcoCalMaxCntEn << ENDVCOCAL_MAXCNT_EN_SHIFT) & ENDVCOCAL_MAXCNT_EN_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES",
            base + ADRV9001_ADDR_CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET,
            bfValue);

        /* Set Quick Cal thresholds */
        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_THRESHOLD(base, 0x3u);
        bfValue = Q_FREQ_CAL_THRESHOLD_VALUE;
        bfValue |= ((pllStateSettings->quickFreqCalEn << Q_FREQ_CAL_EN_SHIFT) & Q_FREQ_CAL_EN_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_QUICK_FREQ_CAL_CTL",
            base + ADRV9001_ADDR_CLK_PLL_QUICK_FREQ_CAL_CTL_OFFSET,
            bfValue);

#if 0   /* this is for Tokelau */
        if (prodId < PRODUCT_ID_B0)
        {
            /* A0 h/w doesn't have this register */
        }
        else
#endif
        {
            //WRITE_CLK_PLL_VCO_BIAS_DAC_LVL_SHIFT_2MA_PMOS_EN(base, 0u);
            bfValue = ((0 << VCO_2MA_PMOS_EN_SHIFT) & VCO_2MA_PMOS_EN_MASK);
            bfValue |= 0x01; /* RESET VALUE */
            ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE10",
                base + ADRV9001_ADDR_CLK_PLL_BYTE10_OFFSET,
                bfValue);
        }

    }
    else
    {
        /* vco_fine_cal_en=0
           vco_cal_init_del<1:0>=2
           vco_cal_alc_init_wait<1:0>=0
           vco_cal_alc_clk_div<1:0>=0
           vco_cal_alc_wait<2:0>=0
           force_vco_init_alc_value=0
           vco_bias_dac_lvl_shift_2ma_pmos_en=1
           vco_out_lvl<3:0> = 4 (override user setting for Power Mode if > 4)
           vco_var<2:0> = 7 - already set in PLL_ComputeSynthParameters
           freq_cal_max_cnt<23:0> =   - already set in PLL_ComputeSynthParameters
        */
        //WRITE_CLK_PLL_VCO_COARSE_CAL_EN(base, 1u);
        //WRITE_CLK_PLL_VCO_FINE_CAL_EN(base, 0u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_WAIT(base, 0u);
        bfValue = VCO_CAL_ALC_WAIT_RESET;
        bfValue |= VCO_COARSE_CAL_EN_MASK;
        bfValue &= ~VCO_FINE_CAL_EN_MASK;
        bfValue |= ((0u << VCO_CAL_ALC_WAIT_SHIFT) & VCO_CAL_ALC_WAIT_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0",
            base + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0_OFFSET,
            bfValue);

        //WRITE_CLK_PLL_VCO_CAL_INIT_DEL(base, 2u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_INIT_WAIT(base, 0u);
        //WRITE_CLK_PLL_VCO_CAL_ALC_CLK_DIV(base, 0u);
        bfValue = VCO_CAL_ALC_STEP_RESET;
        bfValue |= ((2u << VCO_CAL_INIT_DEL_SHIFT) & VCO_CAL_INIT_DEL_MASK);
        bfValue |= ((0u << VCO_CAL_ALC_INIT_WAIT_SHIFT) & VCO_CAL_ALC_INIT_WAIT_MASK);
        bfValue |= ((0u << VCO_CAL_ALC_CLK_DIV_SHIFT) & VCO_CAL_ALC_CLK_DIV_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1",
            base + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1_OFFSET,
            bfValue);

        //WRITE_CLK_PLL_FORCE_VCO_INIT_ALC_VALUE(base, 1u);
        bfValue = FORCE_VCO_INIT_ALC_VALUE;
        bfValue |= (pllStateSettings->vcoCalMaxCntBandEn << VCOCAL_MAXCNTBAND_EN_SHIFT) & VCOCAL_MAXCNTBAND_EN_MASK;
        bfValue |= ((pllStateSettings->endVcoCalMaxCntEn << ENDVCOCAL_MAXCNT_EN_SHIFT) & ENDVCOCAL_MAXCNT_EN_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET",
            base + ADRV9001_ADDR_CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET,
            bfValue);

        /* Set Quick Cal thresholds */
        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_THRESHOLD(base, 0x2u);
        bfValue = Q_FREQ_CAL_THRESHOLD_VALUE;
        bfValue |= ((pllStateSettings->quickFreqCalEn << Q_FREQ_CAL_EN_SHIFT) & Q_FREQ_CAL_EN_MASK);
        ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_QUICK_FREQ_CAL_CTL",
            base + ADRV9001_ADDR_CLK_PLL_QUICK_FREQ_CAL_CTL_OFFSET,
            bfValue);

#if 0   /* this is for Tokelau */
        if (prodId < PRODUCT_ID_B0)
        {
            /* A0 h/w doesn't have this register */
        }
        else
#endif
        {
            //WRITE_CLK_PLL_VCO_BIAS_DAC_LVL_SHIFT_2MA_PMOS_EN(base, 1u);
            bfValue = ((1 << VCO_2MA_PMOS_EN_SHIFT) & VCO_2MA_PMOS_EN_MASK);
            bfValue |= 0x01; /* RESET VALUE */
            ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE10",
                base + ADRV9001_ADDR_CLK_PLL_BYTE10_OFFSET,
                bfValue);
        }
    }
#endif
#endif /* Navassa: moved to PLL_InitStaticRegisters() */

    /* Start the timeout timer */
    /* GPT_Start(timer, SYNTH_CAL_WAIT_US, ONE_SHOT, GPT_INTERRUPT_OFF); */

    /* Start VCO CAL and the timer */
    /* WRITE_CLK_PLL_VCO_CAL_INIT(base, DEF_TRUE); */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_BYTE0",
        base + ADRV9001_ADDR_CLK_PLL_BYTE0_OFFSET,
        (VCO_CAL_INIT_BYTE_RESET | VCO_CAL_INIT_SET));

#if CHECK_VCO_CAL > 0
    /* Wait for a transistion from 0 to 1 which is the cal starting */
    for (timeOut = 0u; timeOut < ADI_ADRV9001_SYNTH_CAL_WAIT_US; timeOut++)
    {
        /* check if CAL is started, or timer expired. */
        /* calBusy = READ_CLK_PLL_VCO_CAL_BUSY(base); */
        ADRV9001_SPIREADBYTE(device, "CLK_PLL_VCO_BAND_BYTE1",
            base + ADRV9001_ADDR_CLK_PLL_VCO_BAND_BYTE1_OFFSET,
            &bfValue);

        calBusy = (bfValue & VCO_CAL_BUSY_MASK) >> VCO_CAL_BUSY_SHIFT;

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            calBusy = 1;
        }
#endif
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
            calBusy = 1;
#endif
        if (calBusy == 1u)
        {
            break;
        }

        /* GPT_DelayUs(1u); */
        halError = adi_common_hal_Wait_us(&device->common, 1);

        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllRunVCOCal()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (timeOut == ADI_ADRV9001_SYNTH_CAL_WAIT_US)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer timeout in adrv9001_PllRunVCOCal()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* If no error, wait for a 1 to 0 transistion */
    /* Wait until calibration is complete or timer expires */
    for (timeOut = 0u; timeOut < ADI_ADRV9001_SYNTH_CAL_WAIT_US; timeOut++)
    {
        /* check if CAL is completed, or timer expired. */
        /* calBusy = READ_CLK_PLL_VCO_CAL_BUSY(base); */
        ADRV9001_SPIREADBYTE(device, "CLK_PLL_VCO_BAND_BYTE1",
            base + ADRV9001_ADDR_CLK_PLL_VCO_BAND_BYTE1_OFFSET,
            &bfValue);
        calBusy = (bfValue & VCO_CAL_BUSY_MASK) >> VCO_CAL_BUSY_SHIFT;

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            calBusy = 0;
        }
#endif
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
            calBusy = 0;
#endif
        if(calBusy == 0u)
        {
            break;
        }

        /* GPT_DelayUs(1u); */
        halError = adi_common_hal_Wait_us(&device->common, 1);

        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllRunVCOCal()");
            ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (timeOut == ADI_ADRV9001_SYNTH_CAL_WAIT_US)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer timeout in adrv9001_PllRunVCOCal()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif
    ADI_API_RETURN(device);
}

/**
* \brief     Update temp compensation in PLL.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllTempUpdateTempComp(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    int16_t  tcidac = 0, tchip = 0;
    int32_t   working, tcidac_f = 0;
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
#ifdef __KERNEL__
    static const uint8_t TCUPDINIT_SET  = 0x02;
#endif
    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    /* The temperature sensor returns the measurement results in (Kevins + TEMP_SENSOR_DFL_OFFSET_CODE_VALUE)
       when TEMP_SLOPE_DEF slope coefficient is used. */

    /* Get the current temperature reading */
    /* PLLTEMP_GetPllTemperature(); */
    tchip = pllStateSettings->temperature;

    /* convert temperature to the TCIdac setting: These equations are empirically generated
       and supplied by the H/W group based on data collected over temperature. */
    working = tchip - ADRV9001_TEMP_SENSOR_DFL_OFFSET_CODE_VALUE + ADRV9001_KELVIN_TO_CELSIUS;

    /*
     * tcidac_f = ADRV9001_TCIDAC_125C + (ADRV9001_TCIDAC_MAX *.84f / 165.0f * (125.0f - working));
     * Rational approximation: 0.84/165 = 7 / 1375
     */

    tcidac_f = ADRV9001_TCIDAC_125C +
               DIV_ROUND_CLOSEST(((ADRV9001_TCIDAC_MAX * (125 - working)) * 7), 1375);

    /* Need to clip tcidac to max/min extreems and convert to int value */
    if (tcidac_f <= 0)
    {
        tcidac = 0;
    }
    else if (tcidac_f >= 4095)
    {
        tcidac = 4095;
    }
    else
    {
        tcidac = (int16_t)(tcidac_f);
    }

    /* Write temp into TCIDAC Low word reg TCIDAC_0 */
    /* WRITE_PMM_VCO_TC_CAL_REGS_TCIDAC_0_TCIDAC(base, (tcidac & 0xFFu)); */
    /* Write temp into TCIDAC High word reg TCIDAC_1 */
    /* WRITE_PMM_VCO_TC_CAL_REGS_TCIDAC_1_TCIDAC(base, (tcidac >> 8u)); */

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_TCIDAC_0",
        baseAddr + ADRV9001_ADDR_CLK_PLL_TCIDAC_0_OFFSET,
        (tcidac & 0xFFu));

    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_TCIDAC_1",
        baseAddr + ADRV9001_ADDR_CLK_PLL_TCIDAC_1_OFFSET,
        (tcidac >> 8u));

#ifndef __KERNEL__
    static const uint8_t TCUPDINIT_SET  = 0x02;
#endif
    /* Write TCUPDINIT reg TCCTL */
    /* WRITE_PMM_TCUPDINIT(base, 1u); */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_TCCTL",
        baseAddr + ADRV9001_ADDR_CLK_PLL_TCCTL_OFFSET,
        TCUPDINIT_SET);

    ADI_API_RETURN(device);
}

/**
* \brief     Wait for a lock detect.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param base - base address of the PLL
* \param len  - lenght of the cal 0=fine, 16=Coarse
* \param wait - wait time in uSec.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static int32_t adrv9001_PllLockDetect(adi_adrv9001_Device_t *device, uint32_t base, uint8_t len, uint32_t waitUsec)
{
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t timeOut = 0;
    uint8_t  lockDet = 0;
    uint8_t bfValue = 0;
    static const uint8_t SYN_LOCK_MASK  = 0x01;
    static const uint8_t SYN_LOCK_SHIFT = 0;

    /* Keep the old lock method until h/w has a change to work on fast lock.
       Steve doesn't want FastLock enabled yet */

    /* Wait for LOCK bit to end */
    for (timeOut = 0u; timeOut != ADI_ADRV9001_SYNTH_LOCK_WAIT_US; timeOut++)
    {
        /* GPT_DelayUs(1u); */
        halError = adi_common_hal_Wait_us(&device->common, 1);

        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllLockDetect()");
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* lockDet = READ_PMM_SYN_LOCK(base); */
        ADRV9001_SPIREADBYTE(device, "CLK_PLL_VCO_BAND_BYTE1",
            base + ADRV9001_ADDR_CLK_PLL_VCO_BAND_BYTE1_OFFSET,
            &bfValue);
        lockDet = (bfValue >> SYN_LOCK_SHIFT) & SYN_LOCK_MASK;

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            lockDet = 1;
        }
#endif
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
            lockDet = 1;
#endif
        if (lockDet == 1u)
        {
            break;
        }
    }
    /* check if we locked */
    if (timeOut == ADI_ADRV9001_SYNTH_LOCK_WAIT_US)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer timeout in adrv9001_PllLockDetect()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

/**
* \brief      Sets 3 least significant bits in ADRV9001_ADDR_CP_CAL_BYTE5_OFFSET and
*             waits for 1 sec to set this value in the respective register.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION       Function completed successfully, no action required
* \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timeout error
*
*/
static uint32_t adrv9001_CpCalSet(adi_adrv9001_Device_t *device,
    adi_adrv9001_Init_t *init,
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr)
{
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t timeout_us = ADI_ADRV9001_CLKPLL_CPCAL_TIMEOUT_US;
    static const uint8_t CP_CAL_BYTE5    = 0x03;

    ADRV9001_SPIWRITEBYTE(device,
        "CP_CAL_BYTE5",
        baseAddr + ADRV9001_ADDR_CP_CAL_BYTE5_OFFSET,
        CP_CAL_BYTE5);

    /* wait */
    halError = adi_common_hal_Wait_us(&device->common, timeout_us);
    ADI_ERROR_REPORT(&device->common,
        ADI_ADRV9001_SRC_INIT,
        halError,
        ADI_COMMON_ACT_ERR_CHECK_TIMER,
        device,
        "Timer not working in adrv9001_CpCalSet()");

    ADI_API_RETURN(device);
}

/**
* \brief      Waits for a PLL lock for the selected PLL, return error if
*              a timeout occurs.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param pllStateSettings - Pointer to Pll State Settings.
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
static uint32_t adrv9001_PllRfSynthCalAndLocked(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, adrv9001_PllSynthParam_t *pllStateSettings)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t base = 0;

    ADI_NULL_PTR_RETURN(&device->common, pllStateSettings);

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        base = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        base = ADRV9001_BF_CLK_PLL;
    }

    /* Configure the timer */
    /* GPT_Open(TIMER_SYNTH_LOCK); */
#if 0   /* In Navassa, calibrate charge pump when PLL is powered up. */
    /* Run the CP cal */
    rtnVal = PLL_RunCpCal(base, TIMER_SYNTH_LOCK);
#endif
    /* check if we cal'd sucessfully */
    if (recoveryAction == ADI_COMMON_ACT_NO_ACTION)
    {
        /* Update the PLL temp compensation h/w */
        recoveryAction = adrv9001_PllTempUpdateTempComp(device, init, pllStateSettings);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Update the PLL temp compensation h/w ");
        ADI_ERROR_RETURN(device->common.error.newAction);


    }

    /* Do a fine cal if in normal mode */
    if (pllStateSettings->mode == PLL_NORM_MODE)
    {
        /* Run fine VCO Cal */
        recoveryAction = adrv9001_PllRunVCOCal(device, base, PLL_COARSE_FINE_CAL, pllStateSettings);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Run coarse/fine VCO Cal ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else
    {
        /* Run Coarse VCO Cal */
        recoveryAction = adrv9001_PllRunVCOCal(device, base, PLL_COARSE_CAL, pllStateSettings);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Run Coarse VCO Cal ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* check if we cal'd sucessfully */
    if (recoveryAction == ADI_COMMON_ACT_NO_ACTION)
    {
        /* if cal was sucessful, wait for lock, the wait time is EffectLoopBW/10 as defined by H/W */
        /* Wait for LOCK bit to end */
        recoveryAction = adrv9001_PllLockDetect(device, base, 0u, (pllStateSettings->loopFilter.effectiveLoopBW / 10u));
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Wait for LOCK bit to end ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    /* Close the timer */
    /* GPT_Close(TIMER_SYNTH_LOCK); */
    ADI_API_RETURN(device);
}

/**
 * \brief    Main API for controlling the System clock. *
 *
 * \param device Structure pointer to ADRV9001 device data structure
 * \param init   Structure pointer to ADRV9001 init data structure
 * \param waitForTempMeas Flag indicating if this function needs to wait
 *                        for the temperature measurement to be done.
 *
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
/* The PLL code was ported from Navassa Firmware */
static int32_t adrv9001_PllSetSysClock(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, uint8_t waitForTempMeas)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    adrv9001_PllSynthParam_t pllStateSettings = { 0 };
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    uint8_t regVal = 0;

    static const uint32_t LOOP_BW = 350000;
    static const uint32_t PHASE_MARGIN = 75;
    static const uint8_t VOUT_LVL = 10; /* PLL_CLKGEN_PLL  */
    static const uint8_t LO_SYNC_DET_VCO_PD_SET    = 0xF0;

    if ((init->clocks.clkPllMode != ADI_ADRV9001_CLK_PLL_LP_MODE) &&
        (init->clocks.clkPllMode != ADI_ADRV9001_CLK_PLL_HP_MODE))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txChannel,
            "Invalid Clock Pll Mode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    /* Set the VCO to the System VCO supplied by API, also scale from kHz to Hz */
    pllStateSettings.vcoFreq_Hz = KILO_TO_BASE_UNIT((uint64_t)init->clocks.clkPllVcoFreq_kHz);
    pllStateSettings.pllFreq_Hz = pllStateSettings.vcoFreq_Hz;

    /* Set the Loop filter default values for desired PLLS  */
    pllStateSettings.loopBW = LOOP_BW;
    pllStateSettings.phaseMargin = PHASE_MARGIN;
    pllStateSettings.voutLvl = VOUT_LVL;
    pllStateSettings.mode = PLL_NORM_MODE;
    pllStateSettings.isLfUserOverride = 0;
    pllStateSettings.phaseSyncMode = PHSYNC_OFF;

    /* Compute the PLL settings */
    recoveryAction = adrv9001_PllComputeSynthParameters(device, init, &pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error Compute PLL setting");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write the PLL settings to H/W */
    recoveryAction = adrv9001_PllWriteSynthParameters(device, init, &pllStateSettings);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        device->common.error.errCode,
        recoveryAction,
        device,
        "Error Write the PLL setting to HW");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* During boot up need to wait for the temperature measurement to be done
       and read the temperature */
    if (waitForTempMeas == true)
    {
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_LO_SYNC_DET_VCO_PD",
            baseAddr + ADRV9001_ADDR_CLK_PLL_LO_SYNC_DET_VCO_PD_OFFSET,
            LO_SYNC_DET_VCO_PD_SET);

#if WAIT_FOR_TEMP == 0
        recoveryAction = adrv9001_PllWaitForTemp(device, init);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Waiting for Temp");
        ADI_ERROR_RETURN(device->common.error.newAction);
#endif
        recoveryAction =  adrv9001_PllTempReadThermistor(device, init, &pllStateSettings);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Reading Temp");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Set SYNTH_RESET_B value to 0x00 */
    regVal = 0x00;
    ADRV9001_SPIWRITEBYTE(device,
        "SYNTH_RESET_B",
        baseAddr + ADRV9001_ADDR_CLK_PLL_SYNTH_RESETB_OFFSET,
        regVal);

    /* Set SYNTH_RESET_B value to 0xFF */
    regVal = 0xFF;
    ADRV9001_SPIWRITEBYTE(device,
        "SYNTH_RESET_B",
        baseAddr + ADRV9001_ADDR_CLK_PLL_SYNTH_RESETB_OFFSET,
        regVal);

    /* Set SYNTH_LOGEN_RESET value to 0x00 */
    regVal = 0x00;
    ADRV9001_SPIWRITEBYTE(device,
        "SYNTH_LOGEN_RESET",
        baseAddr + ADRV9001_ADDR_CLK_PLL_SYNTH_LOGEN_RESET_OFFSET,
        regVal);

    /* Set SYNTH_LOGEN_RESET value to 0x04 */
    regVal = 0x04;
    ADRV9001_SPIWRITEBYTE(device,
        "SYNTH_LOGEN_RESET",
        baseAddr + ADRV9001_ADDR_CLK_PLL_SYNTH_LOGEN_RESET_OFFSET,
        regVal);

    recoveryAction = adrv9001_CpCalSet(device, init, baseAddr);

    /* Calibrate and wait for lock */
    recoveryAction = adrv9001_PllRfSynthCalAndLocked(device, init, &pllStateSettings);
    if (waitForTempMeas == false)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Calibrate and wait for lock in first CLK PLL");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            device->common.error.errCode,
            recoveryAction,
            device,
            "Error Calibrate and wait for lock in second CLK PLL");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

#if 0
static int32_t adrv9001_PllDebug(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    uint8_t readRegVal = 0;

    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x182B,
        &readRegVal);

    printf("Value of 0x182B: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183A,
        &readRegVal);

    printf("Value of 0x183A: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183B,
        &readRegVal);

    printf("Value of 0x183B: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183C,
        &readRegVal);

    printf("Value of 0x183C: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183D,
        &readRegVal);

    printf("Value of 0x183D: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183E,
        &readRegVal);

    printf("Value of 0x183E: %x \n", readRegVal);

    readRegVal = 0;
    ADRV9001_SPIREADBYTE(device,
        "TOP_CLOCK_CTRL",
        0x183F,
        &readRegVal);

    printf("Value of 0x183F: %x \n", readRegVal);

    ADI_API_RETURN(device);

}
#endif

/**
 * \brief    init the temp h/w block.
 *
 * \param device Structure pointer to ADRV9001 device data structure
 * \param clkFreq Clock Frequency in Hz.
 *
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
static int32_t adrv9001_PllTempInitHardware(adi_adrv9001_Device_t *device, uint32_t clkFreq)
{
    uint16_t clockDiv;
    uint8_t regVal = 0;

    static const uint8_t TEMP_SENSE_CLK_EN = 0x04;
    static const uint8_t TEMP_SENSE_CLK_ARM_SEL = 0x02;
    static const uint8_t AUX_ADC_TEMP_SENSE_PD =  0x01;
    static const uint8_t AUX_ADC_TEMP_SENSE_RESET =  0x08;
    static const uint8_t AUX_ADC_TEMP_SENSE_DIG_RESETB =  0x02;
    static const uint8_t TDEGC_DACIN_REF_OVD = 0x04;
    static const uint8_t TDEGC_INIT = 0x01;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* select temp sense clock source */
    /* WRITE_CORE_1_TEMP_SENSE_CLK_ARM_SEL(INST_SPI_CORE_1, clk_src);	*/
    /* enable temp sense clock */
    /* WRITE_CORE_1_TEMP_SENSE_CLK_EN(INST_SPI_CORE_1, 1u); */

    /* clk_src is REF_CLK */
    regVal = (TEMP_SENSE_CLK_EN & ~TEMP_SENSE_CLK_ARM_SEL);
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CLOCK",
        ADRV9001_ADDR_TEMP_SENSE_CLOCK, regVal);

    /* WRITE_CORE_AUX_ADC_TEMP_SENSE_PD(INST_SPI_CORE, 0u); */
    /* WRITE_CORE_AUX_ADC_TEMP_SENSE_RESET(INST_SPI_CORE, 0u); */
    /* WRITE_CORE_AUX_ADC_TEMP_SENSE_DIG_RESETB(INST_SPI_CORE, 1u); */
    regVal = AUX_ADC_TEMP_SENSE_DIG_RESETB & ~AUX_ADC_TEMP_SENSE_PD & ~AUX_ADC_TEMP_SENSE_RESET;
    ADRV9001_SPIWRITEBYTE(device, "AUX_ADC_TEMP_SENSE_CONTROL",
        ADRV9001_ADDR_AUX_ADC_TEMP_SENSE_CONTROL,
        regVal);

    /* Set flag to indicate the use non-default slope coefficient */
    /* WRITE_CORE_TDEGC_DACIN_REF_OVD(INST_SPI_CORE, 1u); */
    /* Write in TEMP_SENSE_CTL_3 */

    /* Hardware expects double of the k1 slope coefficient to be written to
       the registers */
    /* WRITE_CORE_TEMPERATURE_SENSOR_TEMP_SENSE_CTL_2_TDEGC_DACIN_REF(INST_SPI_CORE,
            ((2 * TEMP_SLOPE_DEF) & 0xFFu)); */
    regVal = ((2 * TEMP_SLOPE_DEF) & 0xFF);
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CTL_2",
        ADRV9001_ADDR_TEMP_SENSE_CTL_2,
        regVal);

    /*WRITE_CORE_TEMPERATURE_SENSOR_TEMP_SENSE_CTL_3_TDEGC_DACIN_REF(INST_SPI_CORE,
        ((2 * TEMP_SLOPE_DEF) >> 8u)); */
    regVal = TDEGC_DACIN_REF_OVD;
    regVal |= (((2 * TEMP_SLOPE_DEF) >> 8u) & 0x03);
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CTL_3",
        ADRV9001_ADDR_TEMP_SENSE_CTL_3,
        regVal);

    /* WRITE_CORE_TDEGC_PTAT(INST_SPI_CORE, ((2 * TEMP_SLOPE_DEF) >> 10u)); */
    regVal = (((2 * TEMP_SLOPE_DEF) >> 10u) << 2) & 0x3C;
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CTL_1",
        ADRV9001_ADDR_TEMP_SENSE_CTL_1,
        regVal);

    /* Calculate temp_sense_clk_div<5:0>. We target TEMP_SENSE_FREQ_TARGET */
    clockDiv = clkFreq / TEMP_SENSE_FREQ_TARGET;
    if (clkFreq % TEMP_SENSE_FREQ_TARGET)
    {
        clockDiv++;
    }

    /* Limit divider to the valid range. */
    if (clockDiv > TEMP_SENSE_MAX_DIV)
    {
        clockDiv = TEMP_SENSE_MAX_DIV;
    }
    else if (clockDiv < TEMP_SENSE_MIN_DIV)
    {
        clockDiv = TEMP_SENSE_MIN_DIV;
    }

    /* WRITE_CORE_1_TEMP_SENSE_CLK_DIV(INST_SPI_CORE_1, clockDiv); */
    regVal = clockDiv;
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CLOCK_DIVIDE",
        ADRV9001_ADDR_TEMP_SENSE_CLOCK_DIVIDE,
        regVal);

    /* Start a temp measurement. This is needed for boot of the Clk PLL. */
    /* WRITE_CORE_TDEGC_INIT(INST_SPI_CORE, 1u); */
    regVal = TDEGC_INIT;
    ADRV9001_SPIWRITEBYTE(device, "TEMP_SENSE_CTL_0",
        ADRV9001_ADDR_TEMP_SENSE_CTL_0,
        regVal);

    ADI_API_RETURN(device);
}

/**
 * \brief    Calculate the cal clock divide ratio
 *
 * \details  This function calculates the cal clock divide radio and
 *              Ccal normalization scale so that the tuner cal clock is
 *              between 250 and 500MHz.
 *
 * \param device Structure pointer to ADRV9001 device data structure
 *
 * \retrun  clk - cal clock structure
 */
adrv9001_AdcTunerCalClk_t adrv9001_CalcCalClkDivRatio(adi_adrv9001_Device_t *device)
{
    static const uint16_t CLK_1105_MHZ = 1105;

    adrv9001_AdcTunerCalClk_t clk = {
        .divRatio = 0u,
        .nScale = 1u
    };

    uint16_t hsDigClk_MHz = (uint16_t)(device->devStateInfo.hsDigClk_kHz / 1000);

    if (hsDigClk_MHz == CLK_1105_MHZ)
    {
        clk.divRatio = 4u; /* hsDigClk_MHz is expected to be either 1105MHz or 737MHz.
                                Divide by 4 to get a calibration engine clock of 276.25MHz*/
    }
    else
    {
        clk.divRatio = 2u; /* hsDigClk_MHz must be 737MHz.
                               Divide by 4 to get a calibration engine clock of 368.5MHz*/
    }

    return clk;
}

/**
* \brief Write cal clock divide ratio
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param [in]  ratio   - cal clock divide ratio (in Navassa as per Nazmy's input
*                        we can only use a factor of 4 for an input digital clock
*                        of 1105.92MHz to get a calibration clock of 1105.92MHz/4=
*                        276.48MHz or a factor of 2 for an input digital clock of
*                        737.28MHz to get a calibration clock of 368.64 MHz).
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_WriteClkDivRatio(adi_adrv9001_Device_t *device,
    adi_adrv9001_Init_t *init, uint8_t ratio)
{
    uint8_t regVal = 0;

    static const uint8_t ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_SHIFT = 5;
    static const uint8_t ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_MASK  = 0x20;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* WRITE_CORE_ADC_TUNER_R_C_CAL_CLK_DIV2_SEL(base, ratio);  */

    regVal = (ratio << ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_SHIFT) & ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_MASK;
    ADRV9001_SPIWRITEBYTE(device, "ENSM_CALIBRATION_CONTROL",
        ADRV9001_ADDR_ENSM_CALIBRATION_CONTROL,
        regVal);

    /* Clk1105 High Speed clock selection */
    /* 0: High Speed digital clock Divided by 2 */
    /* 1: High Speed digital clock */
    if (init->clocks.clk1105Div ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_2)
    {
        regVal = 0;
    }
    else if (init->clocks.clk1105Div ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_1)
    {
        regVal = 1;
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            init->clocks.clk1105Div,
            "Invalid init->clocks.clk1105Div\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADRV9001_SPIWRITEBYTE(device, "HS_CLK_DIV",
        ADRV9001_ADDR_HS_CLK_DIV,
        regVal);

    ADI_API_RETURN(device);
}

/**
* \brief Enable or Disable cal clock
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  en      - 0 disable, 1 enable
* \param [in]  ratio   - cal clock divide ratio (same register as 'en')
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_ClkPowerDown(adi_adrv9001_Device_t *device, uint8_t en, uint8_t ratio)
{
    uint8_t regVal = 0;

    static const uint8_t ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_SHIFT = 5;
    static const uint8_t ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_MASK  = 0x20;
    static const uint8_t ADC_TUNER_R_C_CAL_CLK_EN_SHIFT = 4;
    static const uint8_t ADC_TUNER_R_C_CAL_CLK_EN_MASK  = 0x10;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* WRITE_CORE_ADC_TUNER_R_C_CAL_CLK_EN(base, en); */
                                                    /*FIXME: Not the same as in Tokelau.
                                                    Nazmy suggested this register pending
                                                    confirmation by hardware team (Paul). */

    regVal = (ratio << ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_SHIFT) & ADC_TUNER_R_C_CAL_CLK_DIV2_SEL_MASK;
    regVal |= (en << ADC_TUNER_R_C_CAL_CLK_EN_SHIFT) & ADC_TUNER_R_C_CAL_CLK_EN_MASK;

    ADRV9001_SPIWRITEBYTE(device, "ENSM_CALIBRATION_CONTROL",
        ADRV9001_ADDR_ENSM_CALIBRATION_CONTROL,
        regVal);

    ADI_API_RETURN(device);
}

/**
* \brief Enable or Disable cal circuit
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  en      - 0 disable, 1 enable
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_AnalogPowerDown(adi_adrv9001_Device_t *device, uint8_t en)
{
    uint8_t regVal = 0;

    static const uint8_t RXADC_PD_SHIFT = 1;
    static const uint8_t RXADC_PD_MASK  = 0x02;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* WRITE_CORE_1_RXADC_PD(base, en);  */

    regVal = (en << RXADC_PD_SHIFT) & RXADC_PD_MASK;
    ADRV9001_SPIWRITEBYTE(device, "RXADC_PD",
        ADRV9001_ADDR_RXADC_PD,
        regVal);

    ADI_API_RETURN(device);
}

/**
* \brief Start RCAL and CCAL
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  calMode - CCAL/RCAL/BOTH
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_CalStart(adi_adrv9001_Device_t *device, uint8_t calMode)
{
    uint8_t regVal = 0;

    static const uint8_t RXADC_RCAL_START = 0x20;
    static const uint8_t RXADC_CCAL_START = 0x10;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    if (calMode & BITM_ADC_TUNER_RCAL)
    {
        /* WRITE_CORE_1_RXADC_RCAL_START(base, 1u); */
        regVal |= RXADC_RCAL_START;
    }

    if (calMode & BITM_ADC_TUNER_CCAL)
    {
        /* WRITE_CORE_1_RXADC_CCAL_START(base, 1u); */
         regVal |= RXADC_CCAL_START;
    }

    ADRV9001_SPIWRITEBYTE(device, "RXADC_PD",
        ADRV9001_ADDR_RXADC_PD,
        regVal);

    ADI_API_RETURN(device);
}

/**
* \brief check ADC tuner cal status
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  calMode - CCAL/RCAL/BOTH
* \param [out] calDone (0: in progress, 1: completed)
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_CheckCalStatus(adi_adrv9001_Device_t *device, uint8_t calMode, uint8_t *calDone)
{
    uint8_t regVal = 0;

    static const uint8_t RXADC_RCAL_START = 0x20;
    static const uint8_t RXADC_CCAL_START = 0x10;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, calDone);

    ADRV9001_SPIREADBYTE(device, "RXADC_PD",
        ADRV9001_ADDR_RXADC_PD,
        &regVal);

    if (calMode & BITM_ADC_TUNER_RCAL)
    {
        //status |= READ_CORE_1_RXADC_RCAL_START(base);
        if((regVal & RXADC_RCAL_START) == 0)
        {
            *calDone = true;
        }
    }

    if (calMode & BITM_ADC_TUNER_CCAL)
    {
        //status |= READ_CORE_1_RXADC_CCAL_START(base);
        if ((regVal & RXADC_CCAL_START) == 0)
        {
            *calDone = true;
        }
    }

    ADI_API_RETURN(device);
}

/**
* \brief Read conductance measurement result
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
*
* \return      conductance measured
*/
static int32_t adrv9001_AdcTunerHw_ReadGMeasured(adi_adrv9001_Device_t *device, uint16_t *gCal)
{
    uint8_t regValLow = 0;
    uint8_t regValHigh = 0;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, gCal);

    ADRV9001_SPIREADBYTE(device, "RXADC_G_MEASURED_BYTE0",
        ADRV9001_ADDR_RXADC_G_MEASURED_BYTE0,
        &regValLow);

    ADRV9001_SPIREADBYTE(device, "RXADC_G_MEASURED_BYTE1",
        ADRV9001_ADDR_RXADC_G_MEASURED_BYTE1,
        &regValHigh);

    //gcal_l = (uint16_t)READ_CORE_1_RX_ADC_REGISTERS_RXADC_G_MEASURED_BYTE0_RXADC_G_MEASURED(base);
    //gcal_h = (uint16_t)READ_CORE_1_RX_ADC_REGISTERS_RXADC_G_MEASURED_BYTE1_RXADC_G_MEASURED(base);
    *gCal = (((uint16_t)regValHigh << 8) & 0x0F00) | (regValLow & 0xFF);

    ADI_API_RETURN(device);
}

/**
* \brief Test against scaling limit
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  pScale     - pointer to RCAL or CCAL
* \param [in]  nominalVal - RCAL or CCAL nominal value
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_SelfTest(adi_adrv9001_Device_t *device, uint16_t *pScale, uint16_t nominalVal)
{
    uint16_t limit = nominalVal >> 2; /* 25% */

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, pScale);

    if ((*pScale > (nominalVal + limit)) || (*pScale < (nominalVal - limit)))
    {
        *pScale = 0u; /* reset the scaling factor */
    }

    ADI_API_RETURN(device);
}

/**
* \brief Write conductance scale constant
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  sg      - conductance scale constant
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerHw_WriteGScaleConst(adi_adrv9001_Device_t *device, uint16_t sg)
{
    ADI_API_PRIV_ENTRY_EXPECT(device);

    /*FIXME: can't find this register in Navassa. As per Nazmy it was removed in Navassa*/
#if 0   /* This is Tokelau's code. No corresponding Navassa registers exist in the
                regmap. Nazmy indicated the registers were removed from Navassa.
                These registers store the computed sg constant. Other components
                read these spi registers to do their scaling. Don't know if any hardware uses these
                registers. For now will keep values in software.*/
    uint32_t base = OrxSpiBaseAddr[tunerId];

    WRITE_ORX_RX_ADC_REGISTERS_RXADC_G_SCALE_CONST_BYTE0_RXADC_G_SCALE_CONST(base, (uint8_t)(sg & 0xFFu));
    WRITE_ORX_RX_ADC_REGISTERS_RXADC_G_SCALE_CONST_BYTE1_RXADC_G_SCALE_CONST(base, (uint8_t)(sg >> 8));
#endif

    ADI_API_RETURN(device);
}

/**
* \brief Calculate the PTAT Bias RCAL value
*
* \details  In 65nm, a translinear circuit was used to eliminate the PTAT bias
*           current dependence on an internal poly resistor. In this design,
*           the dependence on the internal poly resistor is removed by using
*           an 8-bit programmable resistor that is set according to the result
*           of the ADC RCAL.
*
*  adc_bias_rcal <7:0> = 128 + rcalDelta
*  ptat_bias_rcal <7:0> = round(56 + rcalDelta/2.3 - 0.35)
*
* \pre This function is private and is not called directly by the user.
*
* \param rcalDelta - measured RCAL delta input
*
* \return      Calculated RCAL PTAT bias value
*/
uint8_t adrv9001_AdcTunerHw_CalculatePtatBiasRcal(int16_t rcalDelta)
{
    rcalDelta -= 128;

    return DIV_ROUND_CLOSEST(55650 + DIV_ROUND_CLOSEST(rcalDelta * 10000U, 23), 1000);
}

/**
* \brief Write VCO PTAT bias RCAL data to all PLLs
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllWriteVcoPtatBiasRcal(adi_adrv9001_Device_t *device)
{
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    uint8_t     ptatBias = 0;
    uint16_t    gMeas = 0;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* Calc Ptat Bias */
    //gMeas = (int16_t)adrv9001_AdcTunerHw_ReadGMeasured();
    ADI_EXPECT(adrv9001_AdcTunerHw_ReadGMeasured, device, &gMeas);
    //ptatBias = AdcTunerHw_CalculatePtatBiasRcal(gMeas);
    ptatBias = adrv9001_AdcTunerHw_CalculatePtatBiasRcal((int16_t)gMeas);

    //WRITE_CLK_PLL_VCO_PTAT_BIAS_RCAL(pllSpiBaseAddr[pll], ptatBias);

    if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    ADRV9001_SPIWRITEBYTE(device, "VCO_PTAT_BIAS_RCAL",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_PTAT_BIAS_RCAL_OFFSET,
            ptatBias);

    ADI_API_RETURN(device);
}

/**
* \brief Wait for the calibration to finish, time-out, and get aborted
*
* \detail A success finish is when rcal_start and ccal_start both get cleared
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param [in]  calMode - CCAL/RCAL/BOTH
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllWaitForCal(adi_adrv9001_Device_t *device, uint8_t calMode)
{
    uint32_t    timeOut = 0u;
    uint8_t     calDone = 0u;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    /* We initiated a cal measurement cycle at boot, need to wait until its
       completed, or timeout */
    for (timeOut = 0u; timeOut < (ADC_TUNER_CAL_TIMEOUT_MSEC * 1000); timeOut++)
    {
        ADI_EXPECT(adrv9001_AdcTunerHw_CheckCalStatus, device, calMode, &calDone);

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            calDone = 1;
        }
#endif
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
        calDone = 1;
#endif
        if (calDone == 1u)
        {
            break;
        }

        /* GPT_DelayUs(1u); */
        halError = adi_common_hal_Wait_us(&device->common, 1);

        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForCal()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (timeOut == (ADC_TUNER_CAL_TIMEOUT_MSEC * 1000))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer timeout in adrv9001_PllWaitForCal()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

/**
* \brief runs the hardware calibration for the adc tuner
*
* \details     This function sets up hardware and runs the adc tuner cal
*              returing the scaling values sc and sg per ADC tuner.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
* \param [in]  clk            - Cal clock structure
* \param [in]  pResult        - pointer to the calibration results
* \param [in]  k              - k = (Cid * Vref)/(Iref)
* \param [in]  calMode        - CCAL/RCAL/BOTH
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_TunerCal(adi_adrv9001_Device_t *device,
    adi_adrv9001_Init_t *init,
    adrv9001_AdcTunerCalClk_t clk,
    adrv9001_AdcTunerResult_t *pResult,
    uint16_t k,
    uint8_t calMode)
{
    uint8_t clkDivBit = 0u;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, init);
    ADI_NULL_PTR_RETURN(&device->common, pResult);

    /* set cal clk in the range of 250-500MHz */
    if (clk.divRatio == 4u)
    {
        clkDivBit = 0u; /* divide digital clock by 4 */
    }
    else if (clk.divRatio == 2u)
    {
        clkDivBit = 1u; /* divide digital clock by 2 */
    }
    else
    {
        /* error in cal clock.*/
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txChannel,
            "Invalid RC Init Calibration Clock");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_EXPECT(adrv9001_AdcTunerHw_WriteClkDivRatio, device, init, clkDivBit);

    /* turn on the cal circuit and cal clock. The receiver channels don't need to be on */
    ADI_EXPECT(adrv9001_AdcTunerHw_AnalogPowerDown, device, 0u);
    ADI_EXPECT(adrv9001_AdcTunerHw_ClkPowerDown, device, 1u, clkDivBit);

    /* GPT_DelayUs(ADC_TUNER_CAL_DELAY_US); */
    halError = adi_common_hal_Wait_us(&device->common, ADC_TUNER_CAL_DELAY_USEC);
    ADI_ERROR_REPORT(&device->common,
        ADI_ADRV9001_SRC_INIT,
        halError,
        ADI_COMMON_ACT_ERR_CHECK_TIMER,
        device,
        "Timer not working in adrv9001_TunerCal()");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* assert rxadc_rcal_start and/or rxadc_ccal_start */
    ADI_EXPECT(adrv9001_AdcTunerHw_CalStart, device, calMode);

    /* Wait for rxadc_rcal_start and/or rxadc_ccal_start to clear (finish)
     * or the calibration to time-out or get aborted.
     */
    /* retStatus = CalWait(calMode); */
    ADI_EXPECT(adrv9001_PllWaitForCal, device, calMode) ;

#if 0  /* Not using CCAL */
#if RC_TUNE_UNIT_TEST == 1
    retStatus = NO_ERROR; /* without hardware function above will always time out& return error.
                              Reset error to allow to proceed with testing. */
#endif
                                  /*
                                   *  a. read N1 from rxadc_n1_count_byte0[7:0], rxadc_n1_count_byte1[2:0]
                                   *  b. read N2 from rxadc_n2_count_byte0[7:0], rxadc_n2_count_byte1[2:0]
                                   *  c. read const_delay from rxadc_cal_control
                                   *  d. compute N = ((N1 + N2)/2) - const_delay
                                   *  e. compute Sc =  Cid / Ccal
                                   *                = (Cid * Vref)/(Iref * Tclk * N) => 8.8 format
                                   *                = (Cid * Vref)/(Iref * Tclk * N) => 8.8 format
                                   *
                                   *    where: Vref = 0.512V \
                                   *           Iref = 5uA     | => (Cid * Vref)/(Iref) = 1.09056 * 10^-6 VF/A
                                   *           Cid  = 10.65pF /     k = 1.09056 ~= 0x0117 = 1.08984375 (0.066% error)
                                   *
                                   *           Sc = (k * hs_dig_freq_mhz / (clk.nScale * 10^6)) / N => 8.8 format
                                   */

    if ((calMode & BITM_ADC_TUNER_CCAL))
    {
        uint32_t n, hsdig_mhz;
        uint32_t n1 = AdcTunerHw_ReadN1Count();
        uint32_t n2 = AdcTunerHw_ReadN2Count();
        uint32_t const_delay = AdcTunerHw_ReadCapConstDelay();

#if RC_TUNE_UNIT_TEST == 1
        n1 = N1_TEST_VAL; /* set up some test values to simulate what hardware will */
        n2 = N2_TEST_VAL; /*   eventually compute. */
        const_delay = DEALY_TEST_VAL;
#endif
        n = ((n1 + n2) >> 1) - const_delay;
        if (n == 0) return (RX_HPADC_RC_INIT_CAL_ERROR_INVALID_N_VAL);

        hsdig_mhz = DeviceProfile.hsDigHz_api /
                    ((uint32_t)ONE_MEG * (uint32_t)clk.nScale);

        pResult->sc = (uint16_t)(((uint32_t)k * hsdig_mhz) / n);
        retStatus = SelfTest(&pResult->sc, CCAL_NOMINAL_VAL);
        AdcTunerHw_WriteCapScaleConst(pResult->sc);
    }
#endif

    /*
     *  a. read gcal from rxadc_g_measured_byte0[7:0],  rxadc_g_measured_byte1[3:0]
     *  b. gid = 128
     *  c. compute sg = gcal/gid (expect values to be around 1, so using 5.11 format)
     */
    if ((calMode & BITM_ADC_TUNER_RCAL))
    {
        uint16_t gcal = 0;
        ADI_EXPECT(adrv9001_AdcTunerHw_ReadGMeasured, device, &gcal);

#if RC_TUNE_UNIT_TEST == 1
        gcal = G_TEST_VAL; /* set up some test values to simulate what hardware will compute.*/
#endif
        pResult->sg = (uint16_t)(((uint32_t)gcal << ADC_SG_SHIFT) / 128u);
        ADI_EXPECT(adrv9001_SelfTest, device, &pResult->sg, RCAL_NOMINAL_VAL);

        ADI_EXPECT(adrv9001_AdcTunerHw_WriteGScaleConst, device, pResult->sg) ;
    }

    /* Power down cal circuit and cal clock */
    /* AdcTunerHw_ClkPowerDown(0u); */
    /* AdcTunerHw_AnalogPowerDown(1u); */
    ADI_EXPECT(adrv9001_AdcTunerHw_ClkPowerDown, device, 0u, clkDivBit);
    ADI_EXPECT(adrv9001_AdcTunerHw_AnalogPowerDown, device, 1u);

    ADI_API_RETURN(device);
}

/**
* \brief Adc Tuner Rcal function.
*
* \detail Call from InitAnalog to run Rcal. The results are in scaling factor
*              array and registers.
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
* \param init   Structure pointer to ADRV9001 init data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_AdcTunerRcal(adi_adrv9001_Device_t *device,	adi_adrv9001_Init_t *init)
{
    adrv9001_AdcTunerCalClk_t clk = adrv9001_CalcCalClkDivRatio(device);
    adrv9001_AdcTunerResult_t temp = { .sc = 0u,
                                       .sg = 0u,
                                     };

    static const uint16_t k_AdcTunerCal = 0x0117u; /* k = (Cid * Vref)/(Iref) in 8.8 format */

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, init);

    /* Run Rcal and store the scaling factor in the register */
    ADI_EXPECT(adrv9001_TunerCal, device, init, clk, &temp, k_AdcTunerCal, (BITM_ADC_TUNER_RCAL | BITM_ADC_TUNER_CAL_AT_STARTUP));

    ADI_API_RETURN(device);
}

/**
* \brief Write VCO ALC(Automatic Levelling Control) data to all PLLs
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllWriteALC(adi_adrv9001_Device_t *device)
{
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    static const uint8_t CLK_PLL_ALC_BYTE0    = 0x00;
    static const uint8_t CLK_PLL_ALC_BYTE1    = 0x40;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    /* Clear ALC value before ALC calibration starts. Otherwise clock PLL will have bad phase noise */

    ADRV9001_SPIWRITEBYTE(device,
        "CLK_PLL_ALC_BYTE0",
        baseAddr + ADRV9001_ADDR_CLK_PLL_ALC_BYTE0,
        CLK_PLL_ALC_BYTE0);

    ADRV9001_SPIWRITEBYTE(device,
        "CLK_PLL_ALC_BYTE1",
        baseAddr + ADRV9001_ADDR_CLK_PLL_ALC_BYTE1,
        CLK_PLL_ALC_BYTE1);

    ADI_API_RETURN(device);
}

/**
* \brief Set PLL modulus for all PLLs
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to ADRV9001 device data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_PllSetModulus(adi_adrv9001_Device_t *device)
{
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    uint8_t val[3] = { 0 };
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t numOfBytes = 3;
    uint8_t numOfModes = 3;

    ADI_API_PRIV_ENTRY_EXPECT(device);

    val[0] = (uint8_t)(ADRV9001_CLK_PLL_MODULUS & 0xFF);
    val[1] = (uint8_t)((ADRV9001_CLK_PLL_MODULUS >>  8u) & 0xFF);
    val[2] = (uint8_t)((ADRV9001_CLK_PLL_MODULUS >> 16u) & 0xFF);

    if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }

    /* Set PLL Modulus for all PLL */

    for (i = 0; i < numOfModes; i++)
    {
        for (j = 0; j < numOfBytes; j++)
        {
            ADRV9001_SPIWRITEBYTE(device,
                "CLK_PLL_SDM_REG_FOR_ALL_MODES",
                baseAddr + ((ADRV9001_ADDR_CLK_PLL_SDM_MODE0_BYTE0 + j) + i*numOfBytes),
                val[j]);
        }
    }

    ADI_API_RETURN(device);
}

/**
* \brief Power up PLL Clock LDO.
* *
* \param device Structure pointer to ADRV9001 device data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_PllPowerupSystemLDOs(adi_adrv9001_Device_t *device)
{
    static const uint8_t CLK_SYNTH_LDO_VOUT_TRIM = 3;

    /*****************************************************************
     *  Powerup the PLL slices  steps outlined by H/W
    *****************************************************************/
   /* Navassa */
    if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_HP_MODE)
    {
        /* power up clock PLL LDO */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE2,
            CLK_SYNTH_LDO_VOUT_TRIM);
    }
    else
    {
        /* power up clock PLL LDO */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LP_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE2,
            CLK_SYNTH_LDO_VOUT_TRIM);
    }

    ADI_API_RETURN(device);
}

/**
* \brief Init PLL static registers depending on PLL mode (normal/fast mode).
*
* \param device Structure pointer to ADRV9001 device data structure
* \param fastMode PLL mode (normal/fast mode).
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_PllInitStaticRegisters(adi_adrv9001_Device_t *device, uint8_t fastMode)
{
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    uint32_t maxCnt = 0;
    uint8_t temp = 0;
    const static uint8_t QUICK_FREQ_CAL_CTL_FAST = 0x82;
    const static uint8_t QUICK_FREQ_CAL_CTL_NORMAL = 0x03;
    const static uint8_t MOD_VCO_CAL_CTL_PLUS_SPARES_FAST = 0x86;
    const static uint8_t MOD_VCO_CAL_CTL_PLUS_SPARES_NORMAL = 0x82;
    const static uint8_t VCO_ALC_FREQ_CAL_BYTE0_FAST = 0x80;
    const static uint8_t VCO_ALC_FREQ_CAL_BYTE0_NORMAL = 0xF8;
    const static uint8_t VCO_ALC_FREQ_CAL_BYTE1_FAST = 0x80;
    const static uint8_t VCO_ALC_FREQ_CAL_BYTE1_NORMAL = 0xFF;
    const static uint8_t CLK_PLL_BYTE10_FAST = 0x05;
    const static uint8_t CLK_PLL_BYTE10_NORMAL = 0x01;
    const static uint8_t VCO_F_FINE_BAND_NORMAL = 0x10;

    if (device->devStateInfo.clkPllMode == ADI_ADRV9001_CLK_PLL_HP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
    }

    if (fastMode == true)
    {
        /* Set freq_cal_max_cnt for Normal Mode */
        maxCnt = PLL_CAL_MAX_CNT_FAST;

        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_EN(base, 1u); /* quick_freq_cal_en */
        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_THRESHOLD(base, 2u); /* quick_freq_cal_threshold */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_QUICK_FREQ_CAL_CTL",
            baseAddr + ADRV9001_ADDR_CLK_PLL_QUICK_FREQ_CAL_CTL_OFFSET,
            QUICK_FREQ_CAL_CTL_FAST);

        //WRITE_PLL_MEM_MAP_ENDVCOCAL_MAXCNT_EN(base, 1u); /* endvcocal_maxcnt_en */
        //WRITE_PLL_MEM_MAP_VCOCAL_MAXCNTBAND_EN(base, 1u); /* vcocal_maxcntband_en */
        //WRITE_PLL_MEM_MAP_FORCE_VCO_INIT_ALC_VALUE(base, 1u); /* force_vco_init_cal_value */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES",
            baseAddr + ADRV9001_ADDR_CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET,
            MOD_VCO_CAL_CTL_PLUS_SPARES_FAST);

        //WRITE_PLL_MEM_MAP_VCO_COARSE_CAL_EN(base, 1u); /* vco_coarse_cal_en */
        //WRITE_PLL_MEM_MAP_VCO_FINE_CAL_EN(base, 0u); /* vco_fine_cal_en */
        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_WAIT(base, 0u); /* vco_cal_alc_wait */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0_OFFSET,
            VCO_ALC_FREQ_CAL_BYTE0_FAST);

        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_INIT_WAIT(base, 0u); /* vco_cal_alc_init_wait *
        //WRITE_PLL_MEM_MAP_VCO_CAL_INIT_DEL(base, 2u); /* vco_cal_init_del */
        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_CLK_DIV(base, 0u); /* vco_cal_alc_clk_div */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1_OFFSET,
            VCO_ALC_FREQ_CAL_BYTE1_FAST);

        //WRITE_PLL_MEM_MAP_VCO_BIAS_DAC_LVL_SHIFT_2MA_PMOS_EN(base, 1u); /* vco_bias_dac_lvl_shift_2ma_pmos_en */
         ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_BYTE10",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE10_OFFSET,
            CLK_PLL_BYTE10_FAST);
    }
    else
    {
        /* Set freq_cal_max_cnt for Normal Mode */
        maxCnt = PLL_CAL_MAX_CNT_NORMAL;

        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_EN(base, 0u); /* quick_freq_cal_en */
        //WRITE_PLL_MEM_MAP_QUICK_FREQ_CAL_THRESHOLD(base, 3u); /* quick_freq_cal_threshold */
         ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_QUICK_FREQ_CAL_CTL",
            baseAddr + ADRV9001_ADDR_CLK_PLL_QUICK_FREQ_CAL_CTL_OFFSET,
            QUICK_FREQ_CAL_CTL_NORMAL);

        //WRITE_PLL_MEM_MAP_ENDVCOCAL_MAXCNT_EN(base, 0u); /* endvcocal_maxcnt_en */
        //WRITE_PLL_MEM_MAP_VCOCAL_MAXCNTBAND_EN(base, 1u); /* vcocal_maxcntband_en */
        //WRITE_PLL_MEM_MAP_FORCE_VCO_INIT_ALC_VALUE(base, 1u); /* force_vco_init_cal_value */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES",
            baseAddr + ADRV9001_ADDR_CLK_PLL_MOD_VCO_CAL_CTL_PLUS_SPARES_OFFSET,
            MOD_VCO_CAL_CTL_PLUS_SPARES_NORMAL);

        //WRITE_PLL_MEM_MAP_VCO_COARSE_CAL_EN(base, 1u); /* vco_coarse_cal_en */
        //WRITE_PLL_MEM_MAP_VCO_FINE_CAL_EN(base, 1u); /* vco_fine_cal_en */
        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_WAIT(base, 7u); /* vco_cal_alc_wait */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE0_OFFSET,
            VCO_ALC_FREQ_CAL_BYTE0_NORMAL);

        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_INIT_WAIT(base, 3u); /* vco_cal_alc_init_wait */
        //WRITE_PLL_MEM_MAP_VCO_CAL_INIT_DEL(base, 3u); /* vco_cal_init_del */
        //WRITE_PLL_MEM_MAP_VCO_CAL_ALC_CLK_DIV(base, 3u); /* vco_cal_alc_clk_div */
         ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_ALC_FREQ_CAL_BYTE1_OFFSET,
            VCO_ALC_FREQ_CAL_BYTE1_NORMAL);

        //WRITE_PLL_MEM_MAP_VCO_F_FINE_BAND(base, 16u); /* vco_f_fine_band */

        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_F_FINE_BAND",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_F_FINE_BAND_OFFSET,
            VCO_F_FINE_BAND_NORMAL);

        //WRITE_PLL_MEM_MAP_VCO_BIAS_DAC_LVL_SHIFT_2MA_PMOS_EN(base, 0u); /* vco_bias_dac_lvl_shift_2ma_pmos_en */
         ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_BYTE10",
            baseAddr + ADRV9001_ADDR_CLK_PLL_BYTE10_OFFSET,
            CLK_PLL_BYTE10_NORMAL);
    }

    /* write frequency calibration max counter */
    temp = maxCnt & 0x0FFu;
    //WRITE_PLL_MEM_MAP_SPARE_REGS_FREQ_CAL_MAX_CNT0_FREQ_CAL_MAX_CNT(base, temp); /* freq_cal_max_cnt */
    ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_FREQ_CAL_MAX_CNT0",
            baseAddr + ADRV9001_ADDR_CLK_PLL_FREQ_CAL_MAX_CNT0_OFFSET,
            temp);
    temp = (maxCnt >> 8) & 0x0FFu;
    //WRITE_PLL_MEM_MAP_SPARE_REGS_FREQ_CAL_MAX_CNT1_FREQ_CAL_MAX_CNT(base, temp);
    ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_FREQ_CAL_MAX_CNT1",
            baseAddr + ADRV9001_ADDR_CLK_PLL_FREQ_CAL_MAX_CNT1_OFFSET,
            temp);
    temp = (maxCnt >> 16) & 0x0FFu;
    //WRITE_PLL_MEM_MAP_SPARE_REGS_FREQ_CAL_MAX_CNT2_FREQ_CAL_MAX_CNT(base, temp);
    ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_FREQ_CAL_MAX_CNT2",
            baseAddr + ADRV9001_ADDR_CLK_PLL_FREQ_CAL_MAX_CNT2_OFFSET,
            temp);

    ADI_API_RETURN(device);
}

int32_t adrv9001_ProgramClkPll(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t regVal = 0;
    static const uint8_t HS_DIG_CLK_ENABLE       = 0x40;
    static const uint8_t HS2REF_CLOCK_SWITCH_ENABLE = 0x20;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, init);

    /*- Using Navassa A0 Example one from slide 10 of document linked in "Description"
      - Sequence:
      - Run temperatureMeasurement with DEV_CLK
      - Program and lock CLK_PLL with ideal R-cal value
      - Enable HS_DIG_CLK to digital: SPI: core_bf.hs_dig_clk_enable = 1'b1
      - Switch to DEV_CLK: SPI: core_bf.hs2ref_clock_switch_enable=1'b1
      - Run R-Cal (only) (May need to set clk1105_clk_sel and adc_tuner_r_c_cal_clk_div2_sel
      - Program and lock CLK_PLL with real R-cal value
      - Switch to HS_DIG_CLK:  SPI:hs2ref_clock_switch_enable=1'b0
      - Disable HS_DIG_CLK to digital: SPI: core_bf.hs_dig_clk_enable = 1'b0
     */

    /* Run temperatureMeasurement with DEV_CLK */
    recoveryAction = adrv9001_PllTempInitHardware(device, KILO_TO_BASE_UNIT(init->clocks.deviceClock_kHz));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Program the HS_DIG_CLK divider that provides the clock to the digital */
    adrv9001_AnalogClockSet(device, init);

    /* Initialize PLL LDOs related parameters */
    adrv9001_PllPowerupSystemLDOs(device);
    adrv9001_PllInitStaticRegisters(device, false);

    /* Set PLL Modulus for all PLL */
    recoveryAction = adrv9001_PllSetModulus(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Clear ALC value before ALC calibration starts. Otherwise clock PLL will have bad phase noise */
    recoveryAction = adrv9001_PllWriteALC(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Program and lock CLK_PLL with ideal R-cal value */
    recoveryAction = adrv9001_PllSetSysClock(device, init, false);
    ADI_ERROR_RETURN(device->common.error.newAction);

    // Enable hs_dig_clk in the digital.
    //core_bf.hs_dig_clk_enable.write(bf_status, 1'b1);
    regVal = HS_DIG_CLK_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "TOP_CLOCK_CTRL",
        ADRV9001_ADDR_TOP_CLOCK_CTRL,
        regVal);

    /* Do the internal MCS */
    adi_adrv9001_Mcs_DigitalInt_Set(device, 2);

    /*** After internal MCS switch to the REF_CLK ***/
    /* Enable HS_DIG_CLK to digital: SPI: core_bf.hs_dig_clk_enable = 1'b1
      - Switch to DEV_CLK: SPI: core_bf.hs2ref_clock_switch_enable=1'b1 */
    regVal = HS_DIG_CLK_ENABLE | HS2REF_CLOCK_SWITCH_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "TOP_CLOCK_CTRL",
        ADRV9001_ADDR_TOP_CLOCK_CTRL,
        regVal);

    // Disable the clock to the digital
    //core_bf.hs_dig_clk_enable.write(bf_status, 1'b0);
     regVal = ~HS_DIG_CLK_ENABLE & HS2REF_CLOCK_SWITCH_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "TOP_CLOCK_CTRL",
        ADRV9001_ADDR_TOP_CLOCK_CTRL,
        regVal);

    /* Run R-Cal (only) (May need to set clk1105_clk_sel and adc_tuner_r_c_cal_clk_div2_sel */
    recoveryAction = adrv9001_AdcTunerRcal(device, init);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* write Rcal results to H/W */
    recoveryAction = adrv9001_PllWriteVcoPtatBiasRcal(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Clear ALC value before ALC calibration starts. Otherwise clock PLL will have bad phase noise */
    recoveryAction = adrv9001_PllWriteALC(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Program and lock CLK_PLL with real R-cal value */
    recoveryAction = adrv9001_PllSetSysClock(device, init, true);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Switch to HS_DIG_CLK:  SPI:hs2ref_clock_switch_enable=1'b0
      - Disable HS_DIG_CLK to digital: SPI: core_bf.hs_dig_clk_enable = 1'b0 */
    regVal = 0 & ~HS_DIG_CLK_ENABLE & ~HS2REF_CLOCK_SWITCH_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "TOP_CLOCK_CTRL",
        ADRV9001_ADDR_TOP_CLOCK_CTRL,
        regVal);

    return recoveryAction;
}

