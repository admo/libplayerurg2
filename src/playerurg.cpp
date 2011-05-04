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

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/if.hpp>

#include <unistd.h>

#include <cstring>
#include <algorithm>

extern PlayerTime* GlobalTime;

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////
// Driver object
//////////////////////////////////////////////////////////////////////////////////////

class UrgDriver : public Driver {
public:
    UrgDriver(ConfigFile* cf, int section);
    ~UrgDriver(void);

    // Implementations of virtual functions
    virtual int Setup(void);
    virtual int Shutdown(void);
    
    // This method will be invoked on each incoming message
    virtual int ProcessMessage(MessageQueue *resp_queue,
            player_msghdr *hdr, void *data);

private:
    // Main function for device thread
    virtual void Main(void);

    bool ReadLaser(void);
    bool setupLaser(void);

    // Urg objects
    qrk::UrgCtrl mUrgCtrl;
    std::vector<long> mUrgReadings;
    
    player_laser_data_t mLaserData;
    player_laser_geom_t mLaserGeom;
    player_laser_config_t mLaserConf;
    
    // Configuration parameters
    bool mPowerOnStartup;
    std::string mDeviceName;
    int mBaudRate;
    
    useconds_t mainSleep;
};



////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////

UrgDriver::UrgDriver(ConfigFile* cf, int section) :
Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LASER_CODE) {
    // Init variables
    memset (&mLaserData, 0, sizeof(mLaserData));
    memset (&mLaserGeom, 0, sizeof(mLaserGeom));
    memset (&mLaserConf, 0, sizeof(mLaserConf));
    
    // Get config
    mLaserConf.min_angle = DTOR(cf->ReadInt(section, "min_angle", -120));
    mLaserConf.max_angle = DTOR(cf->ReadInt(section, "max_angle", 120));
    mLaserConf.resolution = DTOR(0.36); //0.36deg
    mLaserConf.max_range = 1.0; //1m
    mLaserConf.range_res = 1.0 / 1000.0; //1mm

    mDeviceName = cf->ReadString(section, "device", "/dev/ttyACM0");
    mPowerOnStartup = cf->ReadBool(section, "power", true);
    mBaudRate = cf->ReadInt(section, "baudrate", 115200);

    // Set up geometry information
    mLaserGeom.pose.px = cf->ReadTupleLength(section, "pose", 0, 0.0);
    mLaserGeom.pose.py = cf->ReadTupleLength(section, "pose", 1, 0.0);
    mLaserGeom.pose.pa = DTOR(cf->ReadTupleLength(section, "pose", 2, 0.0));
    mLaserGeom.size.sw = cf->ReadTupleLength(section, "size", 0, 0.05);
    mLaserGeom.size.sl = cf->ReadTupleLength(section, "size", 1, 0.05);
    
    mUrgReadings.reserve(1024);
    
    mainSleep = cf->ReadInt(section, "main_sleep", 100);
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
        usleep(mainSleep);
    }
}

int UrgDriver::ProcessMessage(MessageQueue *resp_queue, player_msghdr *hdr, void *data) {
    // Property handlers that need to be done manually 
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG, device_addr)) {
        player_laser_config_t *req = reinterpret_cast<player_laser_config_t*> (data);
        // Setting configuration
        mLaserConf.min_angle = req->min_angle;
        mLaserConf.max_angle = req->max_angle;
        setupLaser();
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_SET_CONFIG, &mLaserConf, hdr->size, NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM, device_addr)) {
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_GEOM,
                &mLaserGeom, sizeof (mLaserGeom), NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG,
            device_addr)) {
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_CONFIG, &mLaserConf, sizeof (player_laser_config_t), NULL);
        return 0;
    }
    return -1;
}

bool UrgDriver::ReadLaser(void) {
    double time;
    GlobalTime->GetTimeDouble(&time);

    if (mUrgCtrl.capture(mUrgReadings) < 0) {
        PLAYER_ERROR("UrgDriver::ReadLaser: Failed to read scan.");
        return false;
    }

    // Set mRangeData structure
    mLaserData.min_angle = mLaserConf.min_angle;
    mLaserData.max_angle = mLaserConf.min_angle;
    mLaserData.max_range = mLaserConf.max_range;
    mLaserData.resolution = mLaserConf.resolution;
    mLaserData.intensity_count = 0;
    ++mLaserData.id;
    mLaserData.ranges_count = mUrgReadings.size();
    
    using namespace boost::lambda;
    constant_type<double>::type factor(constant(1e-3));
    std::transform(mUrgReadings.begin(), mUrgReadings.end(), mLaserData.ranges,
            if_then_else_return(_1 < mUrgCtrl.minDistance(),
                factor * mUrgCtrl.maxDistance(),
                factor * _1));
    
    Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN,
            &mLaserData, sizeof(mLaserData), &time);
    
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

    mLaserData.id = 0;

    StartThread();
    return 0;
}

bool UrgDriver::setupLaser() {
    if (!mUrgCtrl.isConnected())
        return false;
    //TODO należy sprawdzić poprawność zakresów
    
    mUrgCtrl.setCaptureRange(mUrgCtrl.rad2index(mLaserConf.min_angle),
            mUrgCtrl.rad2index(mLaserConf.max_angle));

    mLaserConf.max_range = mUrgCtrl.maxDistance() / 1000.0;

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
