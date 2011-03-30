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

extern "C" {
#include <urg_ctrl.h>
}
#include <libplayercore/playercore.h>

extern PlayerTime* GlobalTime;

const int urgBaudrate = 115200;
const double urgAngleCorrection = DTOR(-18.0);
const double urgRealMinAngle = -2.09439103;
const double urgMinAngle = urgRealMinAngle + urgAngleCorrection;
const double urgRealMaxAngle = 2.09439103;
const double urgMaxAngle = urgRealMaxAngle + urgAngleCorrection;


//////////////////////////////////////////////////////////////////////////////////////
// Driver object
//////////////////////////////////////////////////////////////////////////////////////

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
    bool AllocateDataSpace(void);

    // Configuration parameters
    bool _powerOnStartup;
    double _minAngle, _maxAngle;
    std::string _device;
    int _baudRate;

    // Geometry
    player_laser_geom_t _geom;

    urg_t _urg;

    long *_data;
    int _dataSize;
    int _scanID;

};



////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////

UrgDriver::UrgDriver(ConfigFile* cf, int section) :
Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LASER_CODE),
_data(NULL) {
    // Get config
    _minAngle = cf->ReadFloat(section, "min_angle", urgRealMinAngle) + urgAngleCorrection;
    _maxAngle = cf->ReadFloat(section, "max_angle", urgRealMaxAngle) + urgAngleCorrection;
    _device = cf->ReadString(section, "device", "/dev/ttyACM0");
    _powerOnStartup = cf->ReadBool(section, "power", true);
    _baudRate = cf->ReadInt(section, "baudrate", urgBaudrate);

    // Set up geometry information
    _geom.pose.px = cf->ReadTupleLength(section, "pose", 0, 0.0);
    _geom.pose.py = cf->ReadTupleLength(section, "pose", 1, 0.0);
    _geom.pose.pa = DTOR(cf->ReadTupleLength(section, "pose", 2, 0.0));
    _geom.size.sw = cf->ReadTupleLength(section, "size", 0, 0.05);
    _geom.size.sl = cf->ReadTupleLength(section, "size", 1, 0.05);
}

UrgDriver::~UrgDriver(void) {
    if (_data != NULL)
        delete[] _data;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Driver implementation
/////////////////////////////////////////////////////////////////////////////////////////

bool UrgDriver::AllocateDataSpace(void) {

    if (_data != NULL)
        delete[] _data;

    _dataSize = urg_dataMax(&_urg);
    PLAYER_MSG1(1, "UrgDriver::AllocateDataSpace: Allocating data space: %d", _dataSize);
    if ((_data = new long[_dataSize]) == NULL) {
        PLAYER_ERROR1("UrgDriver::AllocateDataSpace: Failed to allocate space for data readings: ", _dataSize);
        return false;
    }
    return true;
}

void UrgDriver::Main(void) {
    while (true) {
        usleep(200000);
        pthread_testcancel();
        ProcessMessages();

        if (!ReadLaser())
            break;
    }
}

int UrgDriver::ProcessMessage(MessageQueue *resp_queue, player_msghdr *hdr, void *data) {
    // Property handlers that need to be done manually 
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG, this->device_addr)) {
        player_laser_config_t *req = reinterpret_cast<player_laser_config_t *> (data);
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_LASER_REQ_SET_CONFIG, data, hdr->size, NULL);
        return 0;
    }
        // Standard ranger messages
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM,
            device_addr)) {
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_GEOM,
                &_geom, sizeof (_geom), NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG,
            device_addr)) {
        int ret;
        player_laser_config_t rangerConfig;
        urg_parameter_t parameters;
        ret = urg_parameters(&_urg, &parameters);
        rangerConfig.min_angle = _minAngle;
        rangerConfig.max_angle = _maxAngle;
        rangerConfig.resolution = DTOR(0.352); //DTOR (360.0)/parameters.area_total_;
        rangerConfig.max_range = parameters.distance_max_ / 1000.0;
        rangerConfig.range_res = 0.03; // 30mm
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_LASER_REQ_GET_CONFIG, &rangerConfig,
                sizeof (rangerConfig), NULL);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG,
            device_addr)) {
        player_laser_config_t *newParams = reinterpret_cast<player_laser_config_t*> (data);

        //_minAngle = newParams->min_angle;
        //_maxAngle = newParams->max_angle;
        if (!AllocateDataSpace()) {
            PLAYER_ERROR("UrgDriver::ProcessMessage: Failed to allocate space for storing range data.");
            Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_LASER_REQ_GET_CONFIG, NULL, 0, NULL);
            return 0;
        }
        int ret;
        urg_parameter_t parameters;
        ret = urg_parameters(&_urg, &parameters);
        if (ret < 0) {
            PLAYER_ERROR("UrgDriver::ProcessMessage: Library error while changing settings:");
            Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_LASER_REQ_GET_CONFIG, NULL, 0, NULL);
            return 0;
        }
#if 0
        if (_minAngle < urg_index2rad(&_urg, parameters.area_min_)) {
            _minAngle = urg_index2rad(&_urg, parameters.area_min_);
            PLAYER_WARN1("UrgDriver::ProcessMessage: Adjusted min_angle to %lf",_minAngle);
        }
        if (_maxAngle > urg_index2rad(&_urg, parameters.area_max_)) {
            _maxAngle = urg_index2rad(&_urg, parameters.area_max_);
            PLAYER_WARN1("UrgDriver::ProcessMessage: Adjusted max_angle to %lf", _maxAngle);
        }
#endif
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_LASER_REQ_GET_CONFIG, newParams,
                sizeof (*newParams), NULL);
        return 0;
    }
    return -1;
}

bool UrgDriver::ReadLaser(void) {
    player_laser_data_t rangeData;
    double time;
    GlobalTime->GetTimeDouble(&time);
    int ret;
    int n;
    int timestamp;
    int index_first = URG_FIRST;
    int index_last = URG_LAST;
    double angle;

    angle = urgMinAngle;
    for (index_first = URG_FIRST; angle < _minAngle; index_first++) {
        angle += DTOR(0.352);
    }
    angle = urgMinAngle;
    for (index_last = index_first; angle <= _maxAngle; index_last++) {
        angle += DTOR(0.352);
    }

    //  ret = urg_requestData(&_urg, URG_GD, URG_FIRST, URG_LAST);
    ret = urg_requestData(&_urg, URG_GD, index_first, index_last);
    if (ret < 0) {
        PLAYER_ERROR("UrgDriver::ReadLaser: Failed to read scan.");
        return false;
    }
    //n = urg_receiveData(&_urg, _data, _dataSize);
    n = urg_receivePartialData(&_urg, _data, _dataSize, index_first, index_last);
    timestamp = urg_recentTimestamp(&_urg);

    rangeData.min_angle = _minAngle;
    rangeData.max_angle = _maxAngle;
    rangeData.max_range = 5.6;
    rangeData.resolution = DTOR(0.352);
    rangeData.ranges_count = n;
    rangeData.intensity_count = 0;
    rangeData.id = _scanID++;

    for (unsigned int i = 0; i < n; i++)
        rangeData.ranges[i] = 1.0 * _data[i] / 1e3;
    for (unsigned int i = 0; i < n; i++) {
        if (rangeData.ranges[i] < 0.02) {// bledne wyniki, usrendiamy
            double r_0 = 0, r_1 = 0;
            int j, k;
            for (j = i; j < n; j++) // szukamy kolejnego porpawnego
                if (rangeData.ranges[j] > 0.02)
                    break;
            if (j < n) //znalezlismy
                r_1 = rangeData.ranges[j];
            else
                r_1 = 0;
            if (i > 0) // znalezlismy
                r_0 = rangeData.ranges[i - 1];
            else
                r_0 = r_1;
            if (r_1 == 0)
                r_1 = r_0;
            if (r_1 == 0)
                r_1 = r_0 = rangeData.max_range;
            for (k = i; k < j; k++)
                //rangeData.ranges[k]=(r_1+r_0)/2;
                rangeData.ranges[k] = r_0 + (r_1 - r_0)*(k - i) / (j - i);
        }

    }
    Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN,
            reinterpret_cast<void*> (&rangeData), sizeof (rangeData), &time);
    return true;
}

int UrgDriver::Setup(void) {
    int ret;
    // Open the laser
    ret = urg_connect(&_urg, _device.c_str(), _baudRate);
    if (ret < 0) {
        PLAYER_ERROR("UrgDriver::Setup: Failed to connect to laser.");
        return -1;
    }

    // Get the sensor information and check _minAngle and _maxAngle are OK
    urg_parameter_t parameters;
    ret = urg_parameters(&_urg, &parameters);
    if (ret < 0) {
        PLAYER_ERROR("UrgDriver::Setup: Failed obtain laser parameters.");
        return -1;
    }
    
    if (_minAngle-urgAngleCorrection < urg_index2rad(&_urg, parameters.area_min_)) {
        _minAngle = urg_index2rad(&_urg, parameters.area_min_) + urgAngleCorrection;
        PLAYER_WARN1("UrgDriver::Setup: Adjusted min_angle to %lf", _minAngle-urgAngleCorrection);
    }
    if (_maxAngle-urgAngleCorrection > urg_index2rad(&_urg, parameters.area_max_)) {
        _maxAngle = urg_index2rad(&_urg, parameters.area_max_) + urgAngleCorrection;
        PLAYER_WARN1("UrgDriver::Setup: Adjusted max_angle to %lf", _maxAngle-urgAngleCorrection);
    }

    if (!AllocateDataSpace())
        return -1;

    if (_powerOnStartup)
        urg_laserOn(&_urg);

    // Get some laser info
    PLAYER_MSG0(0, "UrgDriver::Setup: Laser sensor information:");
    int LinesMax = 5;
    char buffer[LinesMax][UrgLineWidth];
    char *lines[UrgLineWidth];

    std::string info;
    size_t endOfLine;
    for (int i = 0; i < LinesMax; ++i) {
        lines[i] = buffer[i];
    }

    ret = urg_versionLines(&_urg, lines, LinesMax);
    if (ret < 0) {
        PLAYER_ERROR("UrgDriver::Setup: Failed to obtain laser information data.");
        return -1;
    }

    for (int i = 0; i < LinesMax; ++i) {
        info = lines[i];
        endOfLine = info.find(";");
        info.erase(endOfLine);
        PLAYER_MSG1(0, "UrgDriver::Setup: %s", info.c_str());
    }

    StartThread();
    _scanID = 0;
    return 0;
}

int UrgDriver::Shutdown(void) {
    StopThread();
    urg_disconnect(&_urg);
    if (_data != NULL)
        delete[] _data;
    _data = NULL;

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
