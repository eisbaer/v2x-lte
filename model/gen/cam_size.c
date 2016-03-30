#include <stdio.h>
#include <sys/types.h>
#include <CAM.h>
#include <errno.h>

static int write_out(const void *buffer, size_t size, void *app_key) {
    FILE *out_fp = app_key;
    size_t wrote = fwrite(buffer, 1, size, out_fp);
    return (wrote == size) ? 0 : -1;
}

int main(int ac, char **av) {

    CAM_t *message = calloc(1, sizeof(CAM_t));
    if(!message) {
        perror("calloc() failed");
        exit(1);
    }

    // initialize struct

    message->header.protocolVersion = protocolVersion_currentVersion;
    message->header.messageID = messageID_cam;
    message->header.stationID = 1;

    // basicContainer
    message->cam.camParameters.basicContainer.stationType = StationType_passengerCar;

    message->cam.camParameters.basicContainer.referencePosition.latitude = 100;
    message->cam.camParameters.basicContainer.referencePosition.longitude = 100;
    message->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = 200;
    message->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_alt_020_00;

    // highFrequencyContainer
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = HeadingValue_wgs84North;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_equalOrWithinZeroPointOneDegree;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 1000 * SpeedValue_oneCentimeterPerSec;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_forward;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_tenCentimeters * 48; // 4,77 m
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_tenCentimeters * 19; // 1,83 m
    message->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    // LowFrequencyContainer
    LowFrequencyContainer_t *lfc = calloc(1, sizeof(LowFrequencyContainer_t));
    if(!lfc) {
        perror("calloc() failed");
        exit(1);
    }

    // basicVehicleContainerLowFrequency
    lfc->choice.basicVehicleContainerLowFrequency.vehicleRole = VehicleRole_default;
    lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.buf = malloc(sizeof(unsigned char));
    lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.size = 1;
    lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
    lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

    message->cam.camParameters.lowFrequencyContainer = lfc;



    /*
     * encode
     *
     * Unaligned packed encoding rules (PER) as defined in
     * Recommendation ITU-T X.691/ISO/IEC 8825-2 [4] shall
     * be used for CAM encoding and decoding.
     *
     * ETSI EN 302 637-2 V1.3.2 (2014-11)
     */

    asn_enc_rval_t erv; // encoder return value

    erv = uper_encode(&asn_DEF_CAM, message, write_out, stdout);
    fprintf(stdout, "\n");
    if(erv.encoded == -1) {
        fprintf(stderr, "Cannot encode %s: %s\n", erv.failed_type->name, strerror(errno));
    }

    // print structure
    asn_fprint(stdout, &asn_DEF_CAM, message);
    int bytes = erv.encoded / 8;
    bytes += !!(erv.encoded % 8);
    fprintf(stdout, "Total size: %zD bits  %d bytes\n", erv.encoded, bytes);

    return 0;
}
