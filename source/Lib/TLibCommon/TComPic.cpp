/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPic.cpp
    \brief    picture class
*/

#include "TComPic.h"
#include "SEI.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPic::TComPic()
: m_uiTLayer                              (0)
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_pcPicYuvPred                          (NULL)
, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_bCheckLTMSB                           (false)
{
  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    m_apcPicYuv[i]      = NULL;
  }
}

TComPic::~TComPic()
{
  destroy();
}

#if REDUCED_ENCODER_MEMORY
Void TComPic::create( const TComSPS &sps, const TComPPS &pps, const Bool bCreateEncoderSourcePicYuv, const Bool bCreateForImmediateReconstruction )
#else
Void TComPic::create( const TComSPS &sps, const TComPPS &pps, const Bool bIsVirtual)
#endif
{
  destroy();

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth          = sps.getPicWidthInLumaSamples();
  const Int          iHeight         = sps.getPicHeightInLumaSamples();
  const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
  const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
  const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

#if REDUCED_ENCODER_MEMORY
  m_picSym.create( sps, pps, uiMaxDepth, bCreateForImmediateReconstruction );
  if (bCreateEncoderSourcePicYuv)
#else
  m_picSym.create( sps, pps, uiMaxDepth );
  if (!bIsVirtual)
#endif
  {
    m_apcPicYuv[PIC_YUV_ORG    ]   = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG     ]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
#if REDUCED_ENCODER_MEMORY
  if (bCreateForImmediateReconstruction)
  {
#endif
    m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
#if REDUCED_ENCODER_MEMORY
  }
#endif

  // Create an extra TComPicYuv to store the picture to be displayed
  m_apcPicYuv[PIC_YUV_DSP] = new TComPicYuv;
  m_apcPicYuv[PIC_YUV_DSP]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );

  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;

  xCreateCUModeCount(uiMaxDepth);
}

#if REDUCED_ENCODER_MEMORY
Void TComPic::prepareForEncoderSourcePicYuv()
{
  const TComSPS &sps=m_picSym.getSPS();

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth          = sps.getPicWidthInLumaSamples();
  const Int          iHeight         = sps.getPicHeightInLumaSamples();
  const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
  const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
  const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

  if (m_apcPicYuv[PIC_YUV_ORG    ]==NULL)
  {
    m_apcPicYuv[PIC_YUV_ORG    ]   = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG     ]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
  if (m_apcPicYuv[PIC_YUV_TRUE_ORG    ]==NULL)
  {
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
}

Void TComPic::prepareForReconstruction()
{
  if (m_apcPicYuv[PIC_YUV_REC] == NULL)
  {
    const TComSPS &sps=m_picSym.getSPS();
    const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
    const Int          iWidth          = sps.getPicWidthInLumaSamples();
    const Int          iHeight         = sps.getPicHeightInLumaSamples();
    const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
    const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
    const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

    m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }

  // mark it should be extended
  m_apcPicYuv[PIC_YUV_REC]->setBorderExtension(false);

  m_picSym.prepareForReconstruction();
}

Void TComPic::releaseReconstructionIntermediateData()
{
  m_picSym.releaseReconstructionIntermediateData();
}

Void TComPic::releaseEncoderSourceImageData()
{
  if (m_apcPicYuv[PIC_YUV_ORG    ])
  {
    m_apcPicYuv[PIC_YUV_ORG]->destroy();
    delete m_apcPicYuv[PIC_YUV_ORG];
    m_apcPicYuv[PIC_YUV_ORG] = NULL;
  }
  if (m_apcPicYuv[PIC_YUV_TRUE_ORG    ])
  {
    m_apcPicYuv[PIC_YUV_TRUE_ORG]->destroy();
    delete m_apcPicYuv[PIC_YUV_TRUE_ORG];
    m_apcPicYuv[PIC_YUV_TRUE_ORG] = NULL;
  }
}

Void TComPic::releaseAllReconstructionData()
{
  if (m_apcPicYuv[PIC_YUV_REC    ])
  {
    m_apcPicYuv[PIC_YUV_REC]->destroy();
    delete m_apcPicYuv[PIC_YUV_REC];
    m_apcPicYuv[PIC_YUV_REC] = NULL;
  }
  m_picSym.releaseAllReconstructionData();
}
#endif

Void TComPic::destroy()
{
  m_picSym.destroy();

  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    if (m_apcPicYuv[i])
    {
      m_apcPicYuv[i]->destroy();
      delete m_apcPicYuv[i];
      m_apcPicYuv[i]  = NULL;
    }
  }

  deleteSEIs(m_SEIs);

  xDestroyCUModeCount();
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym();
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getNumberOfCtusInFrame(); uiCUAddr++ )
  {
    TComDataCU* pCtu = pPicSym->getCtu(uiCUAddr);
    pCtu->compressMV();
  }
}

Bool  TComPic::getSAOMergeAvailability(Int currAddr, Int mergeAddr)
{
  Bool mergeCtbInSliceSeg = (mergeAddr >= getPicSym()->getCtuTsToRsAddrMap(getCtu(currAddr)->getSlice()->getSliceCurStartCtuTsAddr()));
  Bool mergeCtbInTile     = (getPicSym()->getTileIdxMap(mergeAddr) == getPicSym()->getTileIdxMap(currAddr));
  return (mergeCtbInSliceSeg && mergeCtbInTile);
}

UInt TComPic::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice)
{
  UInt subStrm;
  const bool bWPPEnabled=pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  const TComPicSym &picSym            = *(getPicSym());

  if ((bWPPEnabled && picSym.getFrameHeightInCtus()>1) || (picSym.getNumTiles()>1)) // wavefronts, and possibly tiles being used.
  {
    if (bWPPEnabled)
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt frameWidthInCtus         = picSym.getFrameWidthInCtus();
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      const UInt numTileColumns           = (picSym.getNumTileColumnsMinus1()+1);
      const TComTile *pTile               = picSym.getTComTile(tileIndex);
      const UInt firstCtuRsAddrOfTile     = pTile->getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / frameWidthInCtus;
      // independent tiles => substreams are "per tile"
      const UInt ctuLine                  = ctuRsAddr / frameWidthInCtus;
      const UInt startingSubstreamForTile =(tileYInCtus*numTileColumns) + (pTile->getTileHeightInCtus()*(tileIndex%numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      subStrm=tileIndex;
    }
  }
  else
  {
    // dependent tiles => substreams are "per frame".
    subStrm = 0;
  }
  return subStrm;
}



/**
 * Gets the number of occurances within the picture of a given CU size and
 *   coding mode.
 */
UInt TComPic::getCUModeCount(TComPic::CU_MODE_T mode, UInt depth) {
  return m_ppuiCuModeCount[static_cast<int>(mode)][depth];
}

/**
 * Traverses the CU quadtree to count the number of occurances of each CU size
 *   and coding mode.
 */
Void TComPic::countCUModes() {
  xResetCUModeCount();

  for (int ctuRsAddr = 0; ctuRsAddr < getNumberOfCtusInFrame(); ctuRsAddr++) {
    xCountCUModes(getCtu(ctuRsAddr), 0, 0);
  }
}

/**
 * Traverses the CU quadtree to count the number of occurances of each CU size
 *   and coding mode.
 */
Void TComPic::xCountCUModes(TComDataCU* ctu, UInt cuPartZAddr, UInt depth) {
  const TComSPS* sps = ctu->getSlice()->getSPS();

  UInt cuPelWidth  = sps->getMaxCUWidth()  >> depth;
  UInt cuPelHeight = sps->getMaxCUHeight() >> depth;

  UInt cuLPelX = ctu->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[cuPartZAddr]];
  UInt cuRPelX = cuLPelX + cuPelWidth - 1;
  UInt cuTPelY = ctu->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[cuPartZAddr]];
  UInt cuBPelY = cuTPelY + cuPelHeight - 1;

  Bool isParentCu = (
    depth < ctu->getDepth(cuPartZAddr) &&
    depth < sps->getLog2DiffMaxMinCodingBlockSize()
  );

  Bool isBoundaryCu = (
    cuRPelX >= sps->getPicWidthInLumaSamples() ||
    cuBPelY >= sps->getPicHeightInLumaSamples()
  );

  if (isParentCu || isBoundaryCu) {
    UInt childCuDepth    = depth + 1;
    UInt partsPerChildCu = ctu->getTotalNumPart() >> (childCuDepth << 1);
    UInt childCuPartZAddr = cuPartZAddr;

    for (int i = 0; i < 4; i++) {
      UInt childCuLPelX = ctu->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[childCuPartZAddr]];
      UInt childCuTPelY = ctu->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[childCuPartZAddr]];

      Bool isChildCuInsideFrame = (
        childCuLPelX < sps->getPicWidthInLumaSamples() &&
        childCuTPelY < sps->getPicHeightInLumaSamples()
      );

      if (isChildCuInsideFrame) {
        xCountCUModes(ctu, childCuPartZAddr, childCuDepth);
      }

      childCuPartZAddr += partsPerChildCu;
    }

    return;
  }

  if (ctu->isSkipped(cuPartZAddr)) {
    m_ppuiCuModeCount[static_cast<int>(CU_MODE_SKIP)][depth]++;
  }
  else if (ctu->getIPCMFlag(cuPartZAddr)) {
    m_ppuiCuModeCount[static_cast<int>(CU_MODE_IPCM)][depth]++;
  }
  else if (ctu->isLosslessCoded(cuPartZAddr)) {
    m_ppuiCuModeCount[static_cast<int>(CU_MODE_LOSSLESS)][depth]++;
  }
  else if (ctu->isInter(cuPartZAddr)) {
    m_ppuiCuModeCount[static_cast<int>(CU_MODE_INTER)][depth]++;
  }
  else /* if (ctu->isIntra(cuPartZAddr)) */ {
    m_ppuiCuModeCount[static_cast<int>(CU_MODE_INTRA)][depth]++;
  }
}

/**
 * Initializes the m_ppuiCuModeCount data structure, which counts the number of
 *   occurances of each CU size and coding mode in the picture.
 * \param depth  The max allowed CU depth
 */
Void TComPic::xCreateCUModeCount(UInt depth) {
  for (int i = 0; i < NUM_CU_MODES; i++) {
    m_ppuiCuModeCount[i] = new UInt[depth]();
  }
}

/**
 * Resets the m_ppuiCuModeCount data structure by setting all counts to zero.
 */
Void TComPic::xResetCUModeCount() {
  UInt maxDepth = getPicSym()->getSPS->getLog2DiffMaxMinCodingBlockSize();
  for (int i = 0; i < NUM_CU_MODES; i++) {
    ::memset(m_ppuiCuModeCount[i], 0, maxDepth*sizeof(UInt));
  }
}

/**
 * Destroys the m_ppuiCuModeCount data structure.
 */
Void TComPic::xDestroyCUModeCount() {
  for (int i = 0; i < NUM_CU_MODES; i++) {
    delete[] m_ppuiCuModeCount[i];
  }
}


//! \}
