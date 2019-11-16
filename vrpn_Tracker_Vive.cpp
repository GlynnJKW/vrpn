#include <stdio.h>  // for fprintf, stderr
#include <string.h> // for NULL, strlen

#include "quat.h" // for Q_RAD_TO_DEG, etc
#include "vrpn_Serial.h" // for vrpn_flush_input_buffer, etc
#include "vrpn_Shared.h" // for timeval, vrpn_SleepMsecs, etc
#include "vrpn_Tracker_Vive.h"

vrpn_Tracker_Vive::vrpn_Tracker_Vive(const char* name, vrpn_Connection* c, int numHMD = 1, int numControllers = 2, int numTrackers = 0)
    : vrpn_Analog(name, c)
    , vrpn_Button_Filter(name, c)
    , vrpn_Tracker(name, c)
{
    _numHMD = numHMD;
    _numControllers = numControllers;
    _numTrackers = numTrackers;

	EVRInitError eError = VRInitError_None;
    /*
    VR_Init (
        arg1: Pointer to EVRInitError type (enum defined in openvr.h)
        arg2: Must be of type EVRApplicationType

            The type of VR Applicaion.  This example uses the SteamVR instance
    that is already running. Because of this, the init function will fail if
    SteamVR is not already running.

            Other EVRApplicationTypes include:
                * VRApplication_Scene - "A 3D application that will be drawing
    an environment.""
                * VRApplication_Overlay - "An application that only interacts
    with overlays or the dashboard.""
                * VRApplication_Utility
    */

    vr_pointer = VR_Init(&eError, VRApplication_Background);

    // If the init failed because of an error
    if (eError != VRInitError_None) {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n",
               VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }


    // Set the parameters in the parent classes
    vrpn_Button::num_buttons = _numControllers * BUTTONS_PER_CONTROLLER;
    vrpn_Analog::num_channel = _numControllers * ANALOGS_PER_CONTROLLER;

    vrpn_gettimeofday(&timestamp, NULL); // Set watchdog now

    // Set the status of the buttons and analogs to 0 to start
    clear_values();
}

vrpn_Tracker_Vive::~vrpn_Tracker_Vive() 
{
    if (vr_pointer != NULL) {
		// VR Shutdown:
		// https://github.com/ValveSoftware/openvr/wiki/API-Documentation#initialization-and-cleanup
		VR_Shutdown();
		vr_pointer = NULL;
	}
}

void vrpn_Tracker_Vive::clear_values(void)
{
    int i;

    for (i = 0; i < vrpn_Button::num_buttons; i++) {
        vrpn_Button::buttons[i] = vrpn_Button::lastbuttons[i] = 0;
    }
    for (i = 0; i < vrpn_Analog::getNumChannels(); i++) {
        vrpn_Analog::channel[i] = vrpn_Analog::last[i] = 0;
    }
}

int vrpn_Tracker_Vive::get_report(void)
{
    // ParseTrackingFrame() is where the tracking and vibration code starts
    ParseTrackingFrame();

    // Define a VREvent
    VREvent_t event;
    while (vr_pointer->PollNextEvent(&event, sizeof(event))) {
        /*
            ProcessVREvent is a function defined in this module.  It returns
           false if the function determines the type of error to be fatal or
           signal some kind of quit.
        */
        if (!ProcessVREvent(event)) {
            // If ProcessVREvent determined that OpenVR quit, print quit message
            return 0;
        }
    }
    vrpn_gettimeofday(&timestamp, NULL);
    report_changes(); // Report updates to VRPN
    return 1;
}

void vrpn_Tracker_Vive::report_changes(vrpn_uint32 class_of_service)
{
    vrpn_Analog::timestamp = timestamp;
    vrpn_Button::timestamp = timestamp;

    vrpn_Analog::report_changes(class_of_service);
    vrpn_Button::report_changes();
}

void vrpn_Tracker_Vive::report(vrpn_uint32 class_of_service)
{
    vrpn_Analog::timestamp = timestamp;
    vrpn_Button::timestamp = timestamp;

    vrpn_Analog::report(class_of_service);
    vrpn_Button::report_changes();
}

// This routine is called each time through the server's main loop. It will
// take a course of action depending on the current status of the Spaceball,
// either trying to reset it or trying to get a reading from it.
void vrpn_Tracker_Vive::mainloop()
{
    server_mainloop();
    get_report();
}

void vrpn_Tracker_Vive::iterateAssignIds()
{
    // Un-assigns the deviceIds and hands of controllers. If they are truely
    // connected, will be re-assigned later in this function
    controllers[0].deviceId = -1;
    controllers[1].deviceId = -1;
    controllers[0].hand = -1;
    controllers[1].hand = -1;

    int numTrackersInitialized = 0;
    int numControllersInitialized = 0;

    for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount;
         i++) // Iterates across all of the potential device indicies
    {
        if (!vr_pointer->IsTrackedDeviceConnected(i))
            continue; // Doesn't use the id if the device isn't connected

        // vr_pointer points to the VRSystem that was in init'ed in the
        // constructor.
        ETrackedDeviceClass trackedDeviceClass =
            vr_pointer->GetTrackedDeviceClass(i);

        // Finding the type of device
        if (trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_HMD) {
            hmdDeviceId = i;
        }
        else if (trackedDeviceClass ==
                     ETrackedDeviceClass::TrackedDeviceClass_Controller &&
                 numControllersInitialized < 2) {
            ControllerData* pC = &(controllers[numControllersInitialized]);

            int sHand = -1;

            ETrackedControllerRole role =
                vr_pointer->GetControllerRoleForTrackedDeviceIndex(i);
            if (role ==
                TrackedControllerRole_Invalid) // Invalid hand is
                                               // actually very common,
                                               // always need to test for
                                               // invalid hand
                                               // (lighthouses have lost
                                               // tracking)
                sHand = 0;
            else if (role == TrackedControllerRole_LeftHand)
                sHand = 1;
            else if (role == TrackedControllerRole_RightHand)
                sHand = 2;
            pC->hand = sHand;
            pC->deviceId = i;

            // Used to get/store property ids for the xy of the pad and the
            // analog reading of the trigger
            for (int x = 0; x < k_unControllerStateAxisCount; x++) {
                int prop = vr_pointer->GetInt32TrackedDeviceProperty(
                    pC->deviceId,
                    (ETrackedDeviceProperty)(Prop_Axis0Type_Int32 + x));

                if (prop == k_eControllerAxis_Trigger)
                    pC->idtrigger = x;
                else if (prop == k_eControllerAxis_TrackPad)
                    pC->idpad = x;
            }
            numControllersInitialized++; // Increment this count so that the
                                         // other controller gets initialized
                                         // after initializing this one
        }
        else if (trackedDeviceClass ==
                 ETrackedDeviceClass::TrackedDeviceClass_GenericTracker) {
            TrackerData* pT = &(trackers[numTrackersInitialized]);
            pT->deviceId = i;
            numTrackersInitialized++;
        }
    }
}

void vrpn_Tracker_Vive::setHands()
{
    for (int z = 0; z < 2; z++) {
        ControllerData* pC = &(controllers[z]);
        if (pC->deviceId < 0 ||
            !vr_pointer->IsTrackedDeviceConnected(pC->deviceId))
            continue;
        int sHand = -1;
        // Invalid hand is actually very common, always need to test for invalid
        // hand (lighthouses have lost tracking)
        ETrackedControllerRole role =
            vr_pointer->GetControllerRoleForTrackedDeviceIndex(pC->deviceId);
        if (role == TrackedControllerRole_Invalid)
            sHand = 0;
        else if (role == TrackedControllerRole_LeftHand)
            sHand = 1;
        else if (role == TrackedControllerRole_RightHand)
            sHand = 2;
        pC->hand = sHand;
    }
}

void vrpn_Tracker_Vive::ParseTrackingFrame()
{
    // Runs the iterateAssignIds() method if...
    if (hmdDeviceId < 0 || // HMD id not yet initialized
        controllers[0].deviceId <
            0 || // One of the controllers not yet initialized
        controllers[1].deviceId < 0 ||
        controllers[0].deviceId ==
            controllers[1].deviceId || // Both controllerData structs store the
                                       // same deviceId
        controllers[0].hand ==
            controllers[1]
                .hand) // TODO: Add function to reassign ids each minute or so
    {
        iterateAssignIds();
    }
    if (_numHMD) HMDCoords();
    if (_numControllers) ControllerCoords();
    if (_numTrackers) TrackerCoords();
}

void vrpn_Tracker_Vive::HMDCoords()
{
    if (!vr_pointer->IsTrackedDeviceConnected(hmdDeviceId)){
        printf("uh oh\n");
		return;
	}

    // TrackedDevicePose_t struct is a OpenVR struct. See line 180 in the
    // openvr.h header.
    TrackedDevicePose_t trackedDevicePose;
    // HmdVector3_t position;
    // HmdVector3_t rotation;
    // if (vr_pointer->IsInputFocusCapturedByAnotherProcess())
    // 		printf( "\nINFO--Input Focus by Another Process");
    vr_pointer->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, 0,
                                                &trackedDevicePose, 1);
    // position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
    // rotation = GetEulerRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
    vive2vprn_body(0, trackedDevicePose.mDeviceToAbsoluteTracking, timestamp);

	// int channel_start = 0;
	// //Position
    // vrpn_Analog::channel[channel_start] = position.v[0];
    // vrpn_Analog::channel[channel_start + 1] = position.v[1];
    // vrpn_Analog::channel[channel_start + 2] = position.v[2];

	// //Rotation
    // vrpn_Analog::channel[channel_start + 3] = rotation.v[0];
    // vrpn_Analog::channel[channel_start + 4] = rotation.v[1];
    // vrpn_Analog::channel[channel_start + 5] = rotation.v[2];
}

void vrpn_Tracker_Vive::ControllerCoords()
{
    setHands();

    TrackedDevicePose_t trackedDevicePose;
    VRControllerState_t controllerState;

    // Loops for each ControllerData struct
    for (int i = 0; i < _numControllers; i++) {
        ControllerData* pC = &(controllers[i]);

        if (pC->deviceId < 0 ||
            !vr_pointer->IsTrackedDeviceConnected(pC->deviceId) ||
            pC->hand <
                /*=  Allow printing coordinates for invalid hand? Yes.*/ 0)
            continue;

        vr_pointer->GetControllerStateWithPose(
            TrackingUniverseStanding, pC->deviceId, &controllerState,
            sizeof(controllerState), &trackedDevicePose);
        // pC->pos = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
        // pC->rot = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
        // HmdVector3_t rotation =
        //     GetEulerRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

        pC->isValid = trackedDevicePose.bPoseIsValid;

        int t = pC->idtrigger;
        int p = pC->idpad;

        // This is the call to get analog button data from the controllers
        pC->trigVal = controllerState.rAxis[t].x;
        pC->padX = controllerState.rAxis[p].x;
        pC->padY = controllerState.rAxis[p].y;

        vive2vprn_body(_numHMD + i, trackedDevicePose.mDeviceToAbsoluteTracking, timestamp);

        int channel_start = i * ANALOGS_PER_CONTROLLER;
        vrpn_Analog::channel[channel_start] = pC->trigVal;
        vrpn_Analog::channel[channel_start + 1] = pC->padX;
        vrpn_Analog::channel[channel_start + 2] = pC->padY;
    }
}

void vrpn_Tracker_Vive::TrackerCoords()
{
    TrackedDevicePose_t trackedDevicePose;
    VRControllerState_t controllerState;

    for (int i = 0; i < _numTrackers; i++) {
        TrackerData* pT = &(trackers[i]);

        if (pT->deviceId < 0 ||
            !vr_pointer->IsTrackedDeviceConnected(pT->deviceId))
            continue;

        vr_pointer->GetControllerStateWithPose(
            TrackingUniverseStanding, pT->deviceId, &controllerState,
            sizeof(controllerState), &trackedDevicePose);
        // pT->pos = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
        // pT->rot = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
        pT->isValid = trackedDevicePose.bPoseIsValid;

        vive2vprn_body(_numHMD + _numControllers + i, trackedDevicePose.mDeviceToAbsoluteTracking, timestamp);
    }
}

bool vrpn_Tracker_Vive::ProcessVREvent(const VREvent_t& event)
{
    bool ret = true;
    switch (event.eventType) {
    case VREvent_TrackedDeviceActivated:
        break;

    case VREvent_TrackedDeviceDeactivated:
        break;

    case VREvent_TrackedDeviceUpdated:
        break;

    case VREvent_DashboardActivated:
        break;

    case VREvent_DashboardDeactivated:
        break;

    case VREvent_ChaperoneDataHasChanged:
        break;

    case VREvent_ChaperoneSettingsHaveChanged:
        break;

    case VREvent_ChaperoneUniverseHasChanged:
        break;

    case VREvent_ApplicationTransitionStarted:
        break;

    case VREvent_ApplicationTransitionNewAppStarted:
        break;

    case VREvent_Quit: {
        ret = false;
    } break;

    case VREvent_ProcessQuit: {
        ret = false;
    } break;

    case VREvent_QuitAborted_UserPrompt: {
        ret = false;
    } break;

    case VREvent_QuitAcknowledged: {
        ret = false;
    } break;

    case VREvent_TrackedDeviceRoleChanged:
        break;

    case VREvent_TrackedDeviceUserInteractionStarted:
        break;

    default:
        if (event.eventType >= 200 &&
            event.eventType <= 203) // Button events range from 200-203
            dealWithButtonEvent(event);
        // Check entire event list starts on line #452:
        // https://github.com/ValveSoftware/openvr/blob/master/headers/openvr.h
    }
    return ret;
}

void vrpn_Tracker_Vive::dealWithButtonEvent(VREvent_t event)
{
    int controllerIndex;        // The index of the controllers[] array that
                                // corresponds with the controller that had a
                                // buttonEvent
    for (int i = 0; i < 2; i++) // Iterates across the array of controllers
    {
        ControllerData* pController = &(controllers[i]);
        if (event.trackedDeviceIndex == pController->deviceId){
            int vrpnButton = buttonMapping(event.data.controller.button) + i * BUTTONS_PER_CONTROLLER;
            if(vrpnButton >= vrpn_Button::num_buttons) return;
            if(event.eventType == 200){
                vrpn_Button::buttons[vrpnButton] = 1;
            }
            else if(event.eventType == 201){
                vrpn_Button::buttons[vrpnButton] = 0;
            }
            // printf(
            //     "BUTTON-E--index=%d deviceId=%d hand=%d button=%d event=%d\n",
            //     i, pController->deviceId, pController->hand,
            //     event.data.controller.button, event.eventType);
        }
            

    }
}

int vrpn_Tracker_Vive::buttonMapping(int viveButton)
{
    if(viveButton == 1){
        return 0;
    }
    else if(viveButton == 2){
        return 1;
    }
    else if(viveButton == 32){
        return 2;
    }
    else if(viveButton == 33){
        return 3;
    }
}

int vrpn_Tracker_Vive::vive2vprn_body(int id, HmdMatrix34_t matrix, struct timeval timestamp){
    // HmdVector3_t position = GetPosition(matrix);
    // HmdQuaternion_t rotation = GetRotation(matrix);
    // HmdVector3_t erotation = GetEulerRotation(matrix);

    // for(int a = 0; a < 3; ++a){
    //     for(int b = 0; b < 4; ++b){
    //         printf("%f, ", matrix.m[a][b]);
    //     }
    //     printf("\n");
    // }
    // printf("\n------------\n");

    // printf("%f, %f, %f\n-----\n", erotation.v[0], erotation.v[1], erotation.v[2]);

    // d_sensor = id;
    // pos[0] = position.v[0];
    // pos[1] = position.v[1];
    // pos[2] = position.v[2];

    // d_quat[0] = rotation.x;
    // d_quat[1] = rotation.y;
    // d_quat[2] = rotation.z;
    // d_quat[3] = rotation.w;

    // q_matrix_type destMatrix;
    // for(int a = 0; a < 4; ++a){
    //     for(int b = 0; b < 4; ++b){
    //         if(a == 3 && b == 3){
    //             destMatrix[a][b] = 1;
    //         }
    //         else if(a == 3){
    //             destMatrix[a][b] = 0;
    //         }
    //         else{
    //             destMatrix[a][b] = matrix.m[a][b];
    //         }
    //     }
    // }

    // q_matrix_type convertMatrix;
    // convertMatrix[0][0] = 1;
    // convertMatrix[1][2] = 1;
    // convertMatrix[2][1] = 1;
    // convertMatrix[3][3] = 1;

    // q_matrix_type convertedMatrix;
    // q_matrix_mult(convertedMatrix, destMatrix, convertMatrix);

    // HmdMatrix34_t convertedHmdMatrix;
    // for(int a = 0; a < 3; ++a){
    //     for(int b = 0; b < 4; ++b){
    //         else{
    //             convertedHmdMatrix.m[a][b] = convertedMatrix[a][b];
    //         }
    //     }
    // }

    HmdVector3_t position = GetPosition(matrix);
    HmdQuaternion_t rotation = GetRotation(matrix);

    d_sensor = id;
    pos[0] = position.v[0];
    pos[1] = -position.v[2];
    pos[2] = position.v[1];

    d_quat[0] = rotation.x;
    d_quat[1] = -rotation.z;
    d_quat[2] = rotation.y;
    d_quat[3] = rotation.w;


    
    if(d_connection){
        char msgbuf[1000];
        int len = vrpn_Tracker::encode_to(msgbuf);

        if(d_connection->pack_message(len, timestamp, position_m_id, d_sender_id, msgbuf, vrpn_CONNECTION_LOW_LATENCY)){
            fprintf(stderr, "vrpn_Tracker_DTrack: cannot write message: tossing.\n");
        }
    }
    return 0;
}


HmdVector3_t vrpn_Tracker_Vive::GetEulerRotation(HmdMatrix34_t matrix) 
{
    HmdVector3_t vector;

    float sy =
        sqrt(matrix.m[0][0] * matrix.m[0][0] + matrix.m[1][0] * matrix.m[1][0]);
    bool singular = sy < 1e-6;

	if (!singular) {
        vector.v[0] = atan2(matrix.m[2][1], matrix.m[2][2]);
        vector.v[1] = atan2(-matrix.m[2][0], sy);
        vector.v[2] = atan2(matrix.m[1][0], matrix.m[0][0]);
    }
    else {
        vector.v[0] = atan2(-matrix.m[1][2], matrix.m[1][1]);
        vector.v[1] = atan2(-matrix.m[2][0], sy);
        vector.v[2] = 0;
    }
	
	return vector;
}

HmdVector3_t vrpn_Tracker_Vive::GetPosition(HmdMatrix34_t matrix)
{
    HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

HmdQuaternion_t vrpn_Tracker_Vive::GetRotation(HmdMatrix34_t matrix)
{
    HmdQuaternion_t q;

    q.w =
        sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x =
        sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y =
        sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z =
        sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}

HmdQuaternion_t vrpn_Tracker_Vive::ProcessRotation(HmdQuaternion_t quat)
{
    HmdQuaternion_t out;
    out.w = 2 * acos(quat.w);
    out.x = quat.x / sin(out.w / 2);
    out.y = quat.y / sin(out.w / 2);
    out.z = quat.z / sin(out.w / 2);

    return out;
}