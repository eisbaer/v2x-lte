# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('v2x-lte', ['core', 'network', 'mobility', 'lte', 'netanim'])
    module.env.append_value('INCLUDES', 'module/gen/')
    module.env.append_value('CCFLAGS', '--system-header-prefix=module/gen/')
    module.source = [
    	'helper/v2x-client-helper.cc', 
    	'model/gn-address.cc', 
    	'model/gn-basic-transport-header.cc', 
    	'model/gn-common-header.cc', 
    	'model/location-table.cc', 
    	'model/v2x-client.cc', 
    	'model/v2x-mobility-model.cc',
    	
    	'model/gen/AccelerationConfidence.c', 'model/gen/AccelerationControl.c', 'model/gen/AccidentSubCauseCode.c', 'model/gen/ActionID.c', 'model/gen/AdverseWeatherCondition-AdhesionSubCauseCode.c', 'model/gen/AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.c', 'model/gen/AdverseWeatherCondition-PrecipitationSubCauseCode.c', 'model/gen/AdverseWeatherCondition-VisibilitySubCauseCode.c', 'model/gen/Altitude.c', 'model/gen/AltitudeConfidence.c', 'model/gen/AltitudeValue.c', 'model/gen/asn_codecs_prim.c', 'model/gen/asn_SEQUENCE_OF.c', 'model/gen/asn_SET_OF.c', 'model/gen/BasicContainer.c', 'model/gen/BasicVehicleContainerHighFrequency.c', 'model/gen/BasicVehicleContainerLowFrequency.c', 'model/gen/ber_decoder.c', 'model/gen/ber_tlv_length.c', 'model/gen/ber_tlv_tag.c', 'model/gen/BIT_STRING.c', 'model/gen/BOOLEAN.c', 'model/gen/CAM.c', 'model/gen/CamParameters.c', 'model/gen/CauseCode.c', 'model/gen/CauseCodeType.c', 'model/gen/CenDsrcTollingZone.c', 'model/gen/CenDsrcTollingZoneID.c', 'model/gen/ClosedLanes.c', 'model/gen/CollisionRiskSubCauseCode.c', 'model/gen/constr_CHOICE.c', 'model/gen/constr_SEQUENCE.c', 'model/gen/constr_SEQUENCE_OF.c', 'model/gen/constr_SET_OF.c', 'model/gen/constr_TYPE.c', 'model/gen/constraints.c', 'model/gen/CoopAwareness.c', 'model/gen/Curvature.c', 'model/gen/CurvatureCalculationMode.c', 'model/gen/CurvatureConfidence.c', 'model/gen/CurvatureValue.c', 'model/gen/DangerousEndOfQueueSubCauseCode.c', 'model/gen/DangerousGoodsBasic.c', 'model/gen/DangerousGoodsContainer.c', 'model/gen/DangerousGoodsExtended.c', 'model/gen/DangerousSituationSubCauseCode.c', 'model/gen/DeltaAltitude.c', 'model/gen/DeltaLatitude.c', 'model/gen/DeltaLongitude.c', 'model/gen/DeltaReferencePosition.c', 'model/gen/der_encoder.c', 'model/gen/DriveDirection.c', 'model/gen/DrivingLaneStatus.c', 'model/gen/EmbarkationStatus.c', 'model/gen/EmergencyContainer.c', 'model/gen/EmergencyPriority.c', 'model/gen/EmergencyVehicleApproachingSubCauseCode.c', 'model/gen/EnergyStorageType.c', 'model/gen/EventHistory.c', 'model/gen/EventPoint.c', 'model/gen/ExteriorLights.c', 'model/gen/GenerationDeltaTime.c', 'model/gen/HardShoulderStatus.c', 'model/gen/HazardousLocation-AnimalOnTheRoadSubCauseCode.c', 'model/gen/HazardousLocation-DangerousCurveSubCauseCode.c', 'model/gen/HazardousLocation-ObstacleOnTheRoadSubCauseCode.c', 'model/gen/HazardousLocation-SurfaceConditionSubCauseCode.c', 'model/gen/Heading.c', 'model/gen/HeadingConfidence.c', 'model/gen/HeadingValue.c', 'model/gen/HeightLonCarr.c', 'model/gen/HighFrequencyContainer.c', 'model/gen/HumanPresenceOnTheRoadSubCauseCode.c', 'model/gen/HumanProblemSubCauseCode.c', 'model/gen/IA5String.c', 'model/gen/InformationQuality.c', 'model/gen/INTEGER.c', 'model/gen/ItineraryPath.c', 'model/gen/ItsPduHeader.c', 'model/gen/LanePosition.c', 'model/gen/LateralAcceleration.c', 'model/gen/LateralAccelerationValue.c', 'model/gen/Latitude.c', 'model/gen/LightBarSirenInUse.c', 'model/gen/Longitude.c', 'model/gen/LongitudinalAcceleration.c', 'model/gen/LongitudinalAccelerationValue.c', 'model/gen/LowFrequencyContainer.c', 'model/gen/NativeEnumerated.c', 'model/gen/NativeInteger.c', 'model/gen/NumberOfOccupants.c', 'model/gen/OCTET_STRING.c', 'model/gen/PathDeltaTime.c', 'model/gen/PathHistory.c', 'model/gen/PathPoint.c', 'model/gen/per_decoder.c', 'model/gen/per_encoder.c', 'model/gen/per_opentype.c', 'model/gen/per_support.c', 'model/gen/PerformanceClass.c', 'model/gen/PosCentMass.c', 'model/gen/PosConfidenceEllipse.c', 'model/gen/PosFrontAx.c', 'model/gen/PositioningSolutionType.c', 'model/gen/PositionOfOccupants.c', 'model/gen/PositionOfPillars.c', 'model/gen/PosLonCarr.c', 'model/gen/PosPillar.c', 'model/gen/PostCrashSubCauseCode.c', 'model/gen/ProtectedCommunicationZone.c', 'model/gen/ProtectedCommunicationZonesRSU.c', 'model/gen/ProtectedZoneID.c', 'model/gen/ProtectedZoneRadius.c', 'model/gen/ProtectedZoneType.c', 'model/gen/PtActivation.c', 'model/gen/PtActivationData.c', 'model/gen/PtActivationType.c', 'model/gen/PublicTransportContainer.c', 'model/gen/ReferencePosition.c', 'model/gen/RelevanceDistance.c', 'model/gen/RelevanceTrafficDirection.c', 'model/gen/RequestResponseIndication.c', 'model/gen/RescueAndRecoveryWorkInProgressSubCauseCode.c', 'model/gen/RescueContainer.c', 'model/gen/RestrictedTypes.c', 'model/gen/RoadType.c', 'model/gen/RoadWorksContainerBasic.c', 'model/gen/RoadworksSubCauseCode.c', 'model/gen/RSUContainerHighFrequency.c', 'model/gen/SafetyCarContainer.c', 'model/gen/SemiAxisLength.c', 'model/gen/SequenceNumber.c', 'model/gen/SignalViolationSubCauseCode.c', 'model/gen/SlowVehicleSubCauseCode.c', 'model/gen/SpecialTransportContainer.c', 'model/gen/SpecialTransportType.c', 'model/gen/SpecialVehicleContainer.c', 'model/gen/Speed.c', 'model/gen/SpeedConfidence.c', 'model/gen/SpeedLimit.c', 'model/gen/SpeedValue.c', 'model/gen/StationarySince.c', 'model/gen/StationaryVehicleSubCauseCode.c', 'model/gen/StationID.c', 'model/gen/StationType.c', 'model/gen/SteeringWheelAngle.c', 'model/gen/SteeringWheelAngleConfidence.c', 'model/gen/SteeringWheelAngleValue.c', 'model/gen/SubCauseCodeType.c', 'model/gen/Temperature.c', 'model/gen/TimestampIts.c', 'model/gen/Traces.c', 'model/gen/TrafficConditionSubCauseCode.c', 'model/gen/TrafficRule.c', 'model/gen/TransmissionInterval.c', 'model/gen/TurningRadius.c', 'model/gen/UTF8String.c', 'model/gen/ValidityDuration.c', 'model/gen/VDS.c', 'model/gen/VehicleBreakdownSubCauseCode.c', 'model/gen/VehicleIdentification.c', 'model/gen/VehicleLength.c', 'model/gen/VehicleLengthConfidenceIndication.c', 'model/gen/VehicleLengthValue.c', 'model/gen/VehicleMass.c', 'model/gen/VehicleRole.c', 'model/gen/VehicleWidth.c', 'model/gen/VerticalAcceleration.c', 'model/gen/VerticalAccelerationValue.c', 'model/gen/WheelBaseVehicle.c', 'model/gen/WMInumber.c', 'model/gen/WrongWayDrivingSubCauseCode.c', 'model/gen/xer_decoder.c', 'model/gen/xer_encoder.c', 'model/gen/xer_support.c', 'model/gen/YawRate.c', 'model/gen/YawRateConfidence.c', 'model/gen/YawRateValue.c']


    module_test = bld.create_ns3_module_test_library('v2x-lte')
    module_test.source = [
        'test/v2x-lte-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'v2x-lte'
    headers.source = [
    	'helper/v2x-client-helper.h',
    	'model/gn-address.h',
    	'model/gn-basic-transport-header.h',
    	'model/gn-common-header.h',
    	'model/location-table.h',
    	'model/v2x-mobility-model.h',
    	'model/v2x-client.h',
    	]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')
      
    # bld.ns3_python_bindings()
