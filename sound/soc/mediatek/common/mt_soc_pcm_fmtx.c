/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_fmtx.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio fmtx data1 playback
 *
 * Author:George
 *
 * -------
 *
 *
 *------------------------------------------------------------------------------ *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"
#include "mt_soc_pcm_platform.h"

static AFE_MEM_CONTROL_T *pMemControl;
static struct snd_dma_buffer *FMTX_Playback_dma_buf;
static unsigned int mPlaybackDramState;
static struct device *mDev;

/*
 *    function implementation
 */

static int mtk_fmtx_probe(struct platform_device *pdev);
static int mtk_pcm_fmtx_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_fmtx_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_fmtx_probe(struct snd_soc_platform *platform);

static int fmtx_hdoutput_control = true;

static const char const *fmtx_HD_output[] = {"Off", "On"};

static const struct soc_enum Audio_fmtx_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fmtx_HD_output), fmtx_HD_output),
};


static int Audio_fmtx_hdoutput_Get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_warn("Audio_AmpR_Get = %d\n", fmtx_hdoutput_control);
	ucontrol->value.integer.value[0] = fmtx_hdoutput_control;
	return 0;
}

static int Audio_fmtx_hdoutput_Set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_warn("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(fmtx_HD_output)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	fmtx_hdoutput_control = ucontrol->value.integer.value[0];
	if (fmtx_hdoutput_control) {
		/* set APLL clock setting */
		EnableApll1(true);
		EnableApll2(true);
		#if 0
		EnableI2SDivPower(AUDIO_APLL1_DIV0, true);
		EnableI2SDivPower(AUDIO_APLL2_DIV0, true);
		#else
		EnableI2SCLKDiv(Soc_Aud_APLL1_DIV, true);
		EnableI2SCLKDiv(Soc_Aud_APLL2_DIV, true);
		#endif
		AudDrv_APLL1Tuner_Clk_On();
		AudDrv_APLL2Tuner_Clk_On();
	} else {
		/* set APLL clock setting */
		EnableApll1(false);
		EnableApll2(false);
		#if 0
		EnableI2SDivPower(AUDIO_APLL1_DIV0, false);
		EnableI2SDivPower(AUDIO_APLL2_DIV0, false);
		#else
		EnableI2SCLKDiv(Soc_Aud_APLL1_DIV, false);
		EnableI2SCLKDiv(Soc_Aud_APLL2_DIV, false);
		#endif
		AudDrv_APLL1Tuner_Clk_Off();
		AudDrv_APLL2Tuner_Clk_Off();
	}
	return 0;
}


static const struct snd_kcontrol_new Audio_snd_fmtx_controls[] = {
	SOC_ENUM_EXT("Audio_FMTX_hd_Switch", Audio_fmtx_Enum[0], Audio_fmtx_hdoutput_Get, Audio_fmtx_hdoutput_Set),
};


static struct snd_pcm_hardware mtk_fmtx_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_RESUME |
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats =      SND_SOC_ADV_MT_FMTS,
	.rates =        SOC_HIGH_USE_RATE,
	.rate_min =     SOC_HIGH_USE_RATE_MIN,
	.rate_max =     SOC_NORMAL_USE_RATE_MAX,
	.channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min =      MIN_PERIOD_SIZE,
	.periods_max =      MAX_PERIOD_SIZE,
	.fifo_size =        0,
};

static int mtk_pcm_fmtx_stop(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/* AFE_BLOCK_T *Afe_Block = &(pMemControl->rBlock); */
	PRINTK_AUD_FMTX("mtk_pcm_fmtx_stop\n");

	irq_remove_user(substream, Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE);

	/* here to turn off digital part */
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_MRG_I2S_OUT);

	/* if (GetMrgI2SEnable() == false) */
	/* { */
	SetMrgI2SEnable(false, runtime->rate);
	/* } */

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MRG_I2S_OUT, false);

	Set2ndI2SOutEnable(false);

	EnableAfe(false);

	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);
	AudDrv_Clk_Off();

	return 0;
}

static snd_pcm_uframes_t mtk_pcm_fmtx_pointer(struct snd_pcm_substream
					      *substream)
{
	return get_mem_frame_index(substream,
		pMemControl, Soc_Aud_Digital_Block_MEM_DL1);
}

static int mtk_pcm_fmtx_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;

	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
	if (AllocateAudioSram(&substream->runtime->dma_addr,	&substream->runtime->dma_area,
		substream->runtime->dma_bytes, substream) == 0) {
		AudDrv_Allocate_DL1_Buffer(mDev, substream->runtime->dma_bytes,
			substream->runtime->dma_addr, substream->runtime->dma_area);
		SetHighAddr(Soc_Aud_Digital_Block_MEM_DL1, false, substream->runtime->dma_addr);
		/* pr_warn("mtk_pcm_hw_params dma_bytes = %d\n",substream->runtime->dma_bytes); */
	} else {
		substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
		substream->runtime->dma_area = FMTX_Playback_dma_buf->area;
		substream->runtime->dma_addr = FMTX_Playback_dma_buf->addr;
		SetHighAddr(Soc_Aud_Digital_Block_MEM_DL1, true, substream->runtime->dma_addr);
		mPlaybackDramState = true;
		AudDrv_Emi_Clk_On();
		set_mem_block(substream, hw_params,
			pMemControl, Soc_Aud_Digital_Block_MEM_DL1);
	}
	/* ------------------------------------------------------- */
	pr_warn("1 dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       substream->runtime->dma_bytes, substream->runtime->dma_area,
	       (long)substream->runtime->dma_addr);

	return ret;
}


static int mtk_pcm_fmtx_hw_free(struct snd_pcm_substream *substream)
{
	PRINTK_AUD_FMTX("mtk_pcm_fmtx_hw_free\n");
	pr_warn("%s substream = %p\n", __func__, substream);
	if (mPlaybackDramState == true) {
		AudDrv_Emi_Clk_Off();
		mPlaybackDramState = false;
	} else
		freeAudioSram((void *)substream);
	return 0;
}


static struct snd_pcm_hw_constraint_list constraints_fmtx_sample_rates = {
	.count = ARRAY_SIZE(soc_fm_supported_sample_rates),
	.list = soc_fm_supported_sample_rates,
	.mask = 0,
};

static int mtk_pcm_fmtx_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	mPlaybackDramState = false;
	mtk_fmtx_hardware.buffer_bytes_max = GetPLaybackSramFullSize();

	pr_warn("mtk_I2S0dl1_hardware.buffer_bytes_max = %zu mPlaybackDramState = %d\n",
	       mtk_fmtx_hardware.buffer_bytes_max, mPlaybackDramState);
	runtime->hw = mtk_fmtx_hardware;

	AudDrv_Clk_On();
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_fmtx_hardware ,
	       sizeof(struct snd_pcm_hardware));
	pMemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL1);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_fmtx_sample_rates);

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	if (ret < 0) {
		pr_err("ret < 0 mtkalsa_fmtx_playback close\n");
		mtk_pcm_fmtx_close(substream);
		return ret;
	}
	/* pr_warn("mtk_pcm_I2S0dl1_open return\n"); */
	return 0;
}



static int mtk_pcm_fmtx_close(struct snd_pcm_substream *substream)
{
	PRINTK_AUD_FMTX("%s\n", __func__);
	/* mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_0); */

	AudDrv_Clk_Off();
	return 0;
}

static int mtk_pcm_fmtx_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int mtk_pcm_fmtx_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	AudDrv_Clk_On();

	/* mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_2); */

	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);
	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
	    runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
					     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_AFE_IO_Block_MRG_I2S_OUT); /* FM Tx only support 16 bit */
	} else {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_16_BIT);
		SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_AFE_IO_Block_MRG_I2S_OUT);
	}

	/* here start digital part */
	SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_MRG_I2S_OUT);

	/* set dl1 sample ratelimit_state */
	SetSampleRate(Soc_Aud_Digital_Block_MEM_DL1, runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_DL1, runtime->channels);

	/* start MRG I2S Out */
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MRG_I2S_OUT, true);
	SetMrgI2SEnable(true, runtime->rate);

	/* start 2nd I2S Out */

	Set2ndI2SOutAttribute(runtime->rate);
	Set2ndI2SOutEnable(true);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);

	/* here to set interrupt */
	irq_add_user(substream,
		     Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE,
		     runtime->rate,
		     runtime->period_size * 2 / 3);

	EnableAfe(true);

	return 0;
}

static int mtk_pcm_fmtx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_warn("mtk_pcm_fmtx_trigger cmd = %d\n", cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_fmtx_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_fmtx_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_fmtx_copy(struct snd_pcm_substream *substream,
			     int channel, snd_pcm_uframes_t pos,
			     void __user *dst, snd_pcm_uframes_t count)
{
	return mtk_memblk_copy(substream, channel, pos, dst, count, pMemControl, Soc_Aud_Digital_Block_MEM_DL1);
}

static int mtk_pcm_fmtx_silence(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos,
				snd_pcm_uframes_t count)
{
	PRINTK_AUD_FMTX("%s\n", __func__);
	return 0; /* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_pcm_fmtx_page(struct snd_pcm_substream *substream,
				      unsigned long offset)
{
	PRINTK_AUD_FMTX("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static struct snd_pcm_ops mtk_fmtx_ops = {
	.open =     mtk_pcm_fmtx_open,
	.close =    mtk_pcm_fmtx_close,
	.ioctl =    snd_pcm_lib_ioctl,
	.hw_params =    mtk_pcm_fmtx_hw_params,
	.hw_free =  mtk_pcm_fmtx_hw_free,
	.prepare =  mtk_pcm_fmtx_prepare,
	.trigger =  mtk_pcm_fmtx_trigger,
	.pointer =  mtk_pcm_fmtx_pointer,
	.copy =     mtk_pcm_fmtx_copy,
	.silence =  mtk_pcm_fmtx_silence,
	.page =     mtk_pcm_fmtx_page,
};

static struct snd_soc_platform_driver mtk_fmtx_soc_platform = {
	.ops        = &mtk_fmtx_ops,
	.pcm_new    = mtk_asoc_pcm_fmtx_new,
	.probe      = mtk_afe_fmtx_probe,
};

static int mtk_fmtx_probe(struct platform_device *pdev)
{
	/* int ret = 0; */
	PRINTK_AUD_FMTX("%s\n", __func__);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_FM_MRGTX_PCM);

	PRINTK_AUD_FMTX("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

	mDev = &pdev->dev;

	return snd_soc_register_platform(&pdev->dev,
					 &mtk_fmtx_soc_platform);
}

static int mtk_asoc_pcm_fmtx_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	PRINTK_AUD_FMTX("%s\n", __func__);
	return ret;
}

static int mtk_afe_fmtx_probe(struct snd_soc_platform *platform)
{
	PRINTK_AUD_FMTX("mtk_afe_afe_probe\n");
	snd_soc_add_platform_controls(platform, Audio_snd_fmtx_controls,
				      ARRAY_SIZE(Audio_snd_fmtx_controls));
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_DL1,
				   Dl1_MAX_BUFFER_SIZE);
	FMTX_Playback_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_DL1);

	return 0;
}

static int mtk_fmtx_remove(struct platform_device *pdev)
{
	PRINTK_AUD_FMTX("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_fmtx_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_pcm_fmtx", },
	{}
};
#endif

static struct platform_driver mtk_fmtx_driver = {
	.driver = {
		.name = MT_SOC_FM_MRGTX_PCM,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_soc_pcm_fmtx_of_ids,
#endif
	},
	.probe = mtk_fmtx_probe,
	.remove = mtk_fmtx_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkfmtx_dev;
#endif

static int __init mtk_soc_platform_init(void)
{
	int ret;

	PRINTK_AUD_FMTX("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtkfmtx_dev = platform_device_alloc(MT_SOC_FM_MRGTX_PCM, -1);
	if (!soc_mtkfmtx_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkfmtx_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkfmtx_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_fmtx_driver);
	return ret;

}
module_init(mtk_soc_platform_init);

static void __exit mtk_soc_platform_exit(void)
{
	PRINTK_AUD_FMTX("%s\n", __func__);

	platform_driver_unregister(&mtk_fmtx_driver);
}
module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
