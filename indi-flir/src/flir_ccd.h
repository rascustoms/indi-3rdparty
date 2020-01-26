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

#pragma once

#include <indiccd.h>
#include <Spinnaker.h>

class FLIRCCD : public INDI::CCD {
    public:
        explicit FLIRCCD(Spinnaker::CameraPtr cam);
        ~FLIRCCD() override;

        const char * getDefaultName() override;

        bool initProperties() override;
        void ISGetProperties(const char * dev) override;
        bool updateProperties() override;

        bool Connect() override;
        bool Disconnect() override;

        bool StartExposure(float duration) override;
        bool AbortExposure() override;

        bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n) override;
        bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;
        bool ISNewText(const char * dev, const char * name, char * texts[], char * names[], int n) override;

    protected:
        void TimerHit() override;
        bool saveConfigItems(FILE *fp) override;

        bool UpdateCCDFrame(int x, int y, int w, int h) override;
        bool UpdateCCDBin(int binx, int biny) override;
        bool UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType) override;

        bool StartStreaming() override;
        bool StopStreaming() override;

    private:
        Spinnaker::CameraPtr cam;
        char name[MAXINDIDEVICE];

        double minDuration;
        double maxDuration;

        int timerID;

        int grabImage();
        bool setupParams();

        friend void ::ISSnoopDevice(XMLEle * root);
        friend void ::ISGetProperties(const char * dev);
        friend void ::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int num);
        friend void ::ISNewText(const char * dev, const char * name, char * texts[], char * names[], int num);
        friend void ::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int num);
        friend void ::ISNewBLOB(const char * dev, const char * name, int sizes[], int blobsizes[], char * blobs[],
                                char * formats[], char * names[], int n);
};