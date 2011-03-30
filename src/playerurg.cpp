/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008,2009,2010,2011
 *     Geoffrey Biggs, Łukasz Małek, Adam Oleksy
 *
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation, either version
 * 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>

#include <urg/UrgCtrl.h>
#include <libplayercore/playercore.h>

#include <boost/smart_ptr/scoped_ptr.hpp>

#include <algorithm>
#include <iostream>

extern PlayerTime* GlobalTime;

//////////////////////////////////////////////////////////////////////////////////////
// Driver object
//////////////////////////////////////////////////////////////////////////////////////

float distLong2Float(long d) {
    return 1.0 * d / 1e3;
}

class UrgDriver : public Driver {
public:
    UrgDriver(ConfigFile* cf, int section);
    ~UrgDriver(void);

    virtual int Setup(void);
    virtual int Shutdown(void);
    virtual int ProcessMessage(MessageQueue *resp_queue,
            player_msghdr *hdr, void *data);

private:
    virtual void Main(void);

    bool ReadLaser(void);
    bool setupLaser(void);

    // Urg object
    qrk::UrgCtrl mUrgCtrl;

    // Configuration parameters
    bool mPowerOnStartup;
    std::string mDeviceName;
    int mBaudRate;
    int mMinAngle;
    int mMaxAngle;

    // Geometry
    player_laser_geom_t mLaserGeom;
    player_laser_config_t mLaserConfig;
    //boost::scoped_ptr<player_laser_data_t> mRangeData;
    player_laser_data_t mRangeData;
    std::vector<long> mUrgData;

    int mScanId;

};



////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////

UrgDriver::UrgDriver(ConfigFile* cf, int section) : //mRangeData(new player_laser_data_t),
Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LASER_CODE) {

    // Get config
    mLaserConfig.min_angle = DTOR(cf->ReadInt(section, "min_angle", -120));
    mLaserConfig.max_angle = DTOR(cf->ReadInt(section, "max_angle", 120));
    mLaserConfig.resolution = DTOR(0.36); //0.36deg
    mLaserConfig.max_range = 1.0; //1m
    mLaserConfig.range_res = 1.0 / 1000.0; //1mm

    mDeviceName = cf->ReadString(section, "device", "/dev/ttyACM0");
    mPowerOnStartup = cf->ReadBool(section, "power", true);
    mBaudRate = cf->ReadInt(section, "baudrate", 115200);

    // Set up geometry information
    mLaserGeom.pose.px = cf->ReadTupleLength(section, "pose", 0, 0.0);
    mLaserGeom.pose.py = cf->ReadTupleLength(section, "pose", 1, 0.0);
    mLaserGeom.pose.pa = DTOR(cf->ReadTupleLength(section, "pose", 2, 0.0));
    mLaserGeom.size.sw = cf->ReadTupleLength(section, "size", 0, 0.05);
    mLaserGeom.size.sl = cf->ReadTupleLength(section, "size", 1, 0.05);
    
    mUrgData.reserve(1024);
}

UrgDriver::~UrgDriver(void) {
    mUrgCtrl.disconnect();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Driver implementation
/////////////////////////////////////////////////////////////////////////////////////////

void UrgDriver::Main(void) {
    while (true) {
        // Check to see if Player is trying to terminate the plugin thread
        pthread_testcancel();

        // Process messages
        ProcessMessages();

        if (!ReadLaser())
            break;
        // Sleep this thread so that it releases system resources to other threads
        usleep(200000);
    }
}

int UrgDriver::ProcessMessage(MessageQueue *resp_queue, player_msghdr *hdr, void *data) {
    // Property handlers that need to be done manually 
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG, device_addr)) {
        player_laser_config_t *req = reinterpret_cast<player_laser_config_t*> (data);
        // Setting configuration
        mMinAngle = mUrgCtrl.index2deg(mUrgCtrl.rad2index(req->min_angle));
        mMaxAngle = mUrgCtrl.index2deg(mUrgCtrl.rad2index(req->max_angle));
        mUrgCtrl.setCaptureRange(mUrgCtrl.deg2index(mMinAngle), mUrgCtrl.deg2index(mMaxAngle));
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_SET_CONFIG, data, hdr->size, NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM, device_addr)) {
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_GEOM,
                &mLaserGeom, sizeof (mLaserGeom), NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG,
            device_addr)) {
        int ret;
        player_laser_config_t rangerConfig;
        rangerConfig.min_angle = DTOR(mMinAngle);
        rangerConfig.max_angle = DTOR(mMaxAngle);
        rangerConfig.resolution =
                rangerConfig.max_range = mUrgCtrl.maxDistance() / 1000.0;
        rangerConfig.range_res = 0.03; // 30mm
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_CONFIG, &rangerConfig, sizeof (rangerConfig), NULL);
        return 0;
    }
    return -1;
}

bool UrgDriver::ReadLaser(void) {
    double time;
    GlobalTime->GetTimeDouble(&time);

    if (mUrgCtrl.capture(mUrgData) < 0) {
        PLAYER_ERROR("UrgDriver::ReadLaser: Failed to read scan.");
        return false;
    }

    // Set mRangeData structure
    mRangeData.min_angle = mLaserConfig.min_angle;
    mRangeData.max_angle = mLaserConfig.min_angle;
    mRangeData.max_range = mLaserConfig.max_range;
    mRangeData.resolution = mLaserConfig.resolution;
    mRangeData.intensity_count = 0;
    ++mRangeData.id;

    mRangeData.ranges_count = 0;
    for (int i = mUrgCtrl.rad2index(mLaserConfig.min_angle); i < mUrgData.size(); ++i) {
        mRangeData.ranges[mRangeData.ranges_count++] = 1.0 * mUrgData[i] / 1e3;
    }
    std::transform(mUrgData.begin() + mUrgCtrl.rad2index(mLaserConfig.min_angle),
    mUrgData.begin() + mUrgData.size(), mRangeData.ranges, distLong2Float);
    
    
    for (unsigned int i = 0; i < mRangeData.ranges_count; i++) {
        if (mRangeData.ranges[i] < 0.02) {// bledne wyniki, usrendiamy
            double r_0 = 0, r_1 = 0;
            int j, k;
            for (j = i; j < mRangeData.ranges_count; j++) // szukamy kolejnego porpawnego
                if (mRangeData.ranges[j] > 0.02)
                    break;
            if (j < mRangeData.ranges_count) //znalezlismy
                r_1 = mRangeData.ranges[j];
            else
                r_1 = 0;
            if (i > 0) // znalezlismy
                r_0 = mRangeData.ranges[i - 1];
            else
                r_0 = r_1;
            if (r_1 == 0)
                r_1 = r_0;
            if (r_1 == 0)
                r_1 = r_0 = mRangeData.max_range;
            for (k = i; k < j; k++)
                //rangeData.ranges[k]=(r_1+r_0)/2;
                mRangeData.ranges[k] = r_0 + (r_1 - r_0)*(k - i) / (j - i);
        }
    }
    
    Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN,
            &mRangeData, sizeof(mRangeData), &time);
    
    return true;
}

int UrgDriver::Setup(void) {
    if (!mUrgCtrl.connect(mDeviceName.c_str(), mBaudRate)) {
        PLAYER_ERROR("UrgDriver::Setup: Failed to connect to laser.");
        return -1;
    }

    mUrgCtrl.setLaserOutput(mPowerOnStartup);

    // Get some laser info
    PLAYER_MSG0(0, "UrgDriver::Setup: Laser sensor information:");
    typedef std::vector<std::string> StringVector;
    StringVector info;
    if (!mUrgCtrl.versionLines(info)) {
        PLAYER_ERROR("UrgDriver::Setup: Failed to obtain laser information data.");
        return -1;
    }

    for (StringVector::const_iterator i = info.begin(); i < info.end(); ++i) {
        PLAYER_MSG1(0, "UrgDriver::Setup: %s", i->c_str());
    }

    setupLaser();

    mRangeData.id = 0;

    StartThread();
    return 0;
}

bool UrgDriver::setupLaser() {
    if (!mUrgCtrl.isConnected())
        return false;

    std::cout << mUrgCtrl.rad2index(mLaserConfig.min_angle) << " " << mUrgCtrl.rad2index(mLaserConfig.max_angle) << "\n";
    mUrgCtrl.setCaptureRange(mUrgCtrl.rad2index(mLaserConfig.min_angle),
            mUrgCtrl.rad2index(mLaserConfig.max_angle));

    mLaserConfig.max_range = mUrgCtrl.maxDistance() / 1000.0;

    return true;
}

int UrgDriver::Shutdown(void) {
    StopThread();
    mUrgCtrl.disconnect();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Driver management functions
////////////////////////////////////////////////////////////////////////////////////////////////////

Driver* HokuyoDriver_Init(ConfigFile* cf, int section) {
    return new UrgDriver(cf, section);
}

void hokuyo_aist_Register(DriverTable* table) {
    char driverName[] = "urg";
    table->AddDriver(driverName, HokuyoDriver_Init);
}

extern "C" {

    int player_driver_init(DriverTable* table) {
        PLAYER_MSG0(0, "Hokuyo driver initialization... ");
        hokuyo_aist_Register(table);
        PLAYER_MSG0(0, "done.");
        return 0;
    }
}
