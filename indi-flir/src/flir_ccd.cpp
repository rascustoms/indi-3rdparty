/*
    Driver type: FLIR Camera INDI Driver

    Copyright (C) 2020 Kieren Rasmussen (rascustoms AT gmail DOT com)

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation; either version 2.1 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this library; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

*/

#include "indidevapi.h"
#include "eventloop.h"
#include <iostream>

#include "flir_ccd.h"

static Spinnaker::SystemPtr spinSystem = nullptr;
static std::list<FLIRCCD *> cameras;

static void cleanup() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        delete *camera;
    }
    if (spinSystem) {
        spinSystem->ReleaseInstance();
    }
}

void ISInit() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    static bool isInit = false;
    if (!isInit) {
        spinSystem = Spinnaker::System::GetInstance();
        Spinnaker::CameraList cams = spinSystem->GetCameras();

        unsigned int numCameras = cams.GetSize();

        if (numCameras == 0) {
            std::cerr << "No cameras found" << std::endl;
            cams.Clear();
            // TODO Log
            return;
        }

        for (unsigned int i = 0; i < numCameras; i++) {
            Spinnaker::CameraPtr pCam = cams.GetByIndex(i);
            FLIRCCD *camera = new FLIRCCD(pCam);
            cameras.push_back(camera);
            // TODO Log
        }

        cams.Clear();

        atexit(cleanup);
        isInit = true;
    }
}

void ISGetProperties(const char * dev)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    ISInit();

    if (cameras.empty())
    {
        // TODO Log
        return;
    }

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        if (dev == nullptr || !strcmp(dev, (*camera)->name))
        {
            (*camera)->ISGetProperties(dev);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int num)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    ISInit();

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        if (dev == nullptr || !strcmp(dev, (*camera)->name))
        {
            (*camera)->ISNewSwitch(dev, name, states, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewText(const char * dev, const char * name, char * texts[], char * names[], int num)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    ISInit();

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        if (dev == nullptr || !strcmp(dev, (*camera)->name))
        {
            (*camera)->ISNewText(dev, name, texts, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewNumber(const char * dev, const char * name, double values[], char * names[], int num)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    ISInit();

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        if (dev == nullptr || !strcmp(dev, (*camera)->name))
        {
            (*camera)->ISNewNumber(dev, name, values, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewBLOB(const char * dev, const char * name, int sizes[], int blobsizes[], char * blobs[], char * formats[],
               char * names[], int n)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle * root)
{
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    ISInit();

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        (*camera)->ISSnoopDevice(root);
    }
}

FLIRCCD::FLIRCCD(Spinnaker::CameraPtr cam) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    this->cam = cam;
    this->cam->Init();

    strcpy(this->name, this->cam->DeviceModelName.GetValue().c_str());

    std::cerr << "Initialised: " << this->name << std::endl;

}

FLIRCCD::~FLIRCCD() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    if(cam) {
        cam->DeInit();
    }
}

const char *FLIRCCD::getDefaultName() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return this->name;
}

bool FLIRCCD::initProperties() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    INDI::CCD::initProperties();

    //TODO check if mono or colour for bayer?
    this->SetCCDCapability(CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_BAYER);
    this->addConfigurationControl();
    this->addDebugControl();
    return true;

}

void FLIRCCD::ISGetProperties(const char *dev) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    INDI::CCD::ISGetProperties(dev);
}

bool FLIRCCD::updateProperties() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    INDI::CCD::updateProperties();

    if (this->cam->IsValid()) {
        this->setupParams();

        this->timerID = SetTimer(POLLMS);
    } else {
        rmTimer(timerID);
    }
    return true;
}

bool FLIRCCD::Connect() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return true;
}

bool FLIRCCD::Disconnect() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return true;
}

bool FLIRCCD::StartExposure(float duration) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    if ((duration * 1000000.) < minDuration) {
        // TODO Log
        std::cerr << "Shutter duration less than minimum" << std::endl;
        duration = (float) minDuration;
    }
    if ((duration * 1000000.) > maxDuration) {
        // TODO log
        std::cerr << "Shutter duration greater than maximum" << std::endl;
        duration = (float) maxDuration;
    }

    if (PrimaryCCD.getFrameType() == INDI::CCDChip::BIAS_FRAME) {
        duration = (float) minDuration;
    }

    // Set Exposure Time
    cam->ExposureTime.SetValue(duration * 1000000.);

    // Trigger Exposure
    cam->BeginAcquisition();

    PrimaryCCD.setExposureDuration(duration);
    InExposure = true;

    return true; // Acquisition status from spinnaker?
}

bool FLIRCCD::AbortExposure() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    cam->AcquisitionAbort();
    InExposure = false;
    return true;
}

bool FLIRCCD::ISNewNumber(const char *dev, const char *name, double *values, char **names, int n) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

bool FLIRCCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char **names, int n) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

bool FLIRCCD::ISNewText(const char *dev, const char *name, char **texts, char **names, int n) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return INDI::CCD::ISNewText(dev, name, texts, names, n);
}

void FLIRCCD::TimerHit() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;

    if(!this->cam->IsValid()) {
        return; // No longer connected
    }

    if (InExposure) {
        PrimaryCCD.setExposureLeft(0);
        InExposure = false;
        grabImage();
    }
    this->timerID = this->SetTimer(POLLMS);
}

bool FLIRCCD::saveConfigItems(FILE *fp) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return CCD::saveConfigItems(fp);
}

bool FLIRCCD::UpdateCCDFrame(int x, int y, int w, int h) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    // TODO Value checking
    cam->Width.SetValue(w);
    cam->Height.SetValue(h);
    cam->OffsetX.SetValue(x);
    cam->OffsetY.SetValue(y);

    PrimaryCCD.setFrame(x, y, w, h);

    int nbuf;
    nbuf = (x * y * PrimaryCCD.getBPP() / 8);
//    nbuf += 512;
    PrimaryCCD.setFrameBufferSize(nbuf);


//    return CCD::UpdateCCDFrame(x, y, w, h);
}

bool FLIRCCD::UpdateCCDBin(int binx, int biny) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return CCD::UpdateCCDBin(binx, biny);
}

bool FLIRCCD::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    INDI::CCDChip::CCD_FRAME imageFrameType = PrimaryCCD.getFrameType();

    if (fType == imageFrameType)
        return true;

    switch (imageFrameType)
    {
        case INDI::CCDChip::BIAS_FRAME:
        case INDI::CCDChip::DARK_FRAME:
            /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Frame type here
     *  BIAS and DARK are taken with shutter closed, so _usually_
     *  most CCD this is a call to let the CCD know next exposure shutter
     *  must be closed. Customize as appropiate for the hardware
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to set frame type to ...");
     *  return false;
     *
     *
     **********************************************************/
            break;

        case INDI::CCDChip::LIGHT_FRAME:
        case INDI::CCDChip::FLAT_FRAME:
            /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Frame type here
     *  LIGHT and FLAT are taken with shutter open, so _usually_
     *  most CCD this is a call to let the CCD know next exposure shutter
     *  must be open. Customize as appropiate for the hardware
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to set frame type to ...");
     *  return false;
     *
     *
     **********************************************************/
            break;
    }

    PrimaryCCD.setFrameType(fType);

    return true;
}

bool FLIRCCD::StartStreaming() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return CCD::StartStreaming();
}

bool FLIRCCD::StopStreaming() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    return CCD::StopStreaming();
}

int FLIRCCD::grabImage() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    Spinnaker::ImagePtr image = this->cam->GetNextImage();
    uint8_t *fb = PrimaryCCD.getFrameBuffer();

    if (PrimaryCCD.getFrameBufferSize() == (int) image->GetBufferSize()) {
        std::memcpy(fb, image->GetData(), image->GetBufferSize());
        image->Release();
        cam->EndAcquisition();
        this->ExposureComplete(&PrimaryCCD);
    } else {
        std::cerr << "Buffer's don't match" << std::endl;
    }

    return 0;
}

bool FLIRCCD::setupParams() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    float x_pixel_size, y_pixel_size;
    int bit_depth;
    int x, y, w, h;

    /** Camera Setup */
    // Set to Mono16
    cam->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono16);
    // Turn off auto exposure
    cam->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
    // Set exposure mode to "Timed"
    cam->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);
    // Disable Frame Rate limiting
    cam->AcquisitionFrameRateEnable.SetValue(false);
    // Set mode to continuous
    cam->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
    // Jumbo Packets
    cam->GevSCPSPacketSize.SetValue(9000);

    // TODO put in model numbers

    x_pixel_size = 5.86;
    y_pixel_size = 5.86;

    switch(cam->PixelSize.GetValue()) {
        case Spinnaker::PixelSize_Bpp1:
            bit_depth = 1;
            break;
        case Spinnaker::PixelSize_Bpp2:
            bit_depth = 2;
            break;
        case Spinnaker::PixelSize_Bpp4:
            bit_depth = 4;
            break;
        case Spinnaker::PixelSize_Bpp8:
            bit_depth = 8;
            break;
        case Spinnaker::PixelSize_Bpp10:
            bit_depth = 10;
            break;
        case Spinnaker::PixelSize_Bpp12:
            bit_depth = 12;
            break;
        case Spinnaker::PixelSize_Bpp14:
            bit_depth = 14;
            break;
        case Spinnaker::PixelSize_Bpp16:
            bit_depth = 16;
            break;
        case Spinnaker::PixelSize_Bpp20:
            bit_depth = 20;
            break;
        case Spinnaker::PixelSize_Bpp24:
            bit_depth = 24;
            break;
        case Spinnaker::PixelSize_Bpp30:
            bit_depth = 30;
            break;
        case Spinnaker::PixelSize_Bpp32:
            bit_depth = 32;
            break;
        case Spinnaker::PixelSize_Bpp36:
            bit_depth = 36;
            break;
        case Spinnaker::PixelSize_Bpp48:
            bit_depth = 48;
            break;
        case Spinnaker::PixelSize_Bpp64:
            bit_depth = 64;
            break;
        case Spinnaker::PixelSize_Bpp96:
            bit_depth = 96;
            break;
        default:
            bit_depth = 8;
            break;
    }

    x = cam->OffsetX.GetMin();
    y = cam->OffsetY.GetMin();
    w = cam->Width.GetMax();
    h = cam->Height.GetMax();

    minDuration = cam->ExposureTime.GetMin();
    maxDuration = cam->ExposureTime.GetMax();

    SetCCDParams(w - x, h - y, bit_depth, x_pixel_size, y_pixel_size);

    int nbuf;
    nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8;
//    nbuf += 512;
    PrimaryCCD.setFrameBufferSize(nbuf);

    switch (cam->PixelColorFilter.GetValue()) {
        case Spinnaker::PixelColorFilter_None:
            SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
            break;
        case Spinnaker::PixelColorFilter_BayerRG: // RGGB
            IUSaveText(&BayerT[2], "RGGB");
            break;
        case Spinnaker::PixelColorFilter_BayerGB: // GBRG
            IUSaveText(&BayerT[2], "GBRG");
            break;
        case Spinnaker::PixelColorFilter_BayerGR: // GRBG
            IUSaveText(&BayerT[2], "GRBG");
            break;
        case Spinnaker::PixelColorFilter_BayerBG: // BGGR
            IUSaveText(&BayerT[2], "BGGR");
            break;
        default:
            SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
            break;
    }

    return true;
}
