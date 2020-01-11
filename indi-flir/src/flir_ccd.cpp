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

static std::list<FLIRCCD *> cameras;

static void cleanup() {
    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        delete *camera;
    }
}

void ISInit() {
    std::cerr << "Do I get here" << std::endl;
    static bool isInit = false;
    if (!isInit) {
        Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
        Spinnaker::CameraList cams = system->GetCameras();
        unsigned int numCameras = cams.GetSize();

        if (numCameras == 0) {
            // TODO Log
            return;
        }

        for (unsigned int i = 0; i < numCameras; i++) {
            Spinnaker::CameraPtr pCam = cams.GetByIndex(i);
            FLIRCCD *camera = new FLIRCCD(pCam);
            cameras.push_back(camera);
            // TODO Log
        }

        system->ReleaseInstance();

        atexit(cleanup);
        isInit = true;
    }
}

void ISGetProperties(const char * dev)
{
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
    ISInit();

    std::list<FLIRCCD *>::iterator camera;
    for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
        (*camera)->ISSnoopDevice(root);
    }
}

FLIRCCD::FLIRCCD(Spinnaker::CameraPtr cam) {
    this->cam = cam;
//    this->setDeviceName(this->cam->DeviceModelName.GetValue());
    cam->Init();
}

FLIRCCD::~FLIRCCD() {

}

const char *FLIRCCD::getDefaultName() {
    return this->name;
}

bool FLIRCCD::initProperties() {
    INDI::CCD::initProperties();

    //TODO check if mono or colour for bayer?
    this->SetCCDCapability(CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_BAYER);
    this->addConfigurationControl();
    this->addDebugControl();
    return true;

}

void FLIRCCD::ISGetProperties(const char *dev) {
    INDI::CCD::ISGetProperties(dev);
}

bool FLIRCCD::updateProperties() {
    INDI::CCD::updateProperties();

    if (this->cam->IsInitialized()) {
        this->setupParams();

        this->timerID = SetTimer(POLLMS);
    } else {
        rmTimer(timerID);
    }
}

bool FLIRCCD::Connect() {
    return true;
}

bool FLIRCCD::Disconnect() {
    return true;
}

bool FLIRCCD::StartExposure(float duration) {

    if (duration < minDuration) {
        //TODO log
        duration = minDuration;
    }

    // TODO Max duration check

    if (PrimaryCCD.getFrameType() == INDI::CCDChip::BIAS_FRAME) {
        duration = minDuration;
    }

    // Set Exposure Time
//    cam->ExposureTime.SetValue(duration * 1000000.);
    // Trigger Exposure
    // TODO

    InExposure = true;

    return true; // Acquisition status from spinnaker?
}

bool FLIRCCD::AbortExposure() {
    cam->AcquisitionAbort();
    InExposure = false;
    return true;
}

bool FLIRCCD::ISNewNumber(const char *dev, const char *name, double *values, char **names, int n) {
    return true;
}

bool FLIRCCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char **names, int n) {
    return true;
}

bool FLIRCCD::ISNewText(const char *dev, const char *name, char **texts, char **names, int n) {
    return true;
}

void FLIRCCD::TimerHit() {
    DefaultDevice::TimerHit();
}

bool FLIRCCD::saveConfigItems(FILE *fp) {
    return CCD::saveConfigItems(fp);
}

bool FLIRCCD::UpdateCCDFrame(int x, int y, int w, int h) {
    return CCD::UpdateCCDFrame(x, y, w, h);
}

bool FLIRCCD::UpdateCCDBin(int binx, int biny) {
    return CCD::UpdateCCDBin(binx, biny);
}

bool FLIRCCD::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType) {
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
    return CCD::StartStreaming();
}

bool FLIRCCD::StopStreaming() {
    return CCD::StopStreaming();
}

bool FLIRCCD::setupParams() {
    float x_pixel_size, y_pixel_size;
    int bit_depth;
    int x, y, w, h;

    // TODO get from spinnaker

    x_pixel_size = 5.86;
    y_pixel_size = 5.86;

    bit_depth = 16;

    x = 0;
    y = 0;
    w = 1920;
    h = 1200;

    minDuration = 0;
    maxDuration = 32000000.;

    SetCCDParams(w - x, h - y, bit_depth, x_pixel_size, y_pixel_size);

    int nbuf;
    nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8;
//    nbuf += 512;
    PrimaryCCD.setFrameBufferSize(nbuf);


    /** Camera Setup */
    // Turn off auto exposure
//    cam->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
    //Set exposure mode to "Timed"
//    cam->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);

    // TODO Setup Camera

    return true;
}
