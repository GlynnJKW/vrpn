#ifndef VRPN_TRACKER_VIVE_H
#define VRPN_TRACKER_VIVE_H

#include "vrpn_Analog.h"     // for vrpn_Serial_Analog
#include "vrpn_Button.h"     // for vrpn_Button_Filter
#include "vrpn_Tracker.h"    // for vrpn_Tracker
#include "vrpn_Configure.h"  // for VRPN_API
#include "vrpn_Connection.h" // for vrpn_CONNECTION_LOW_LATENCY, etc
#include "vrpn_Shared.h"     // for timeval
#include "vrpn_Types.h"      // for vrpn_uint32

#include <iostream>
#include <math.h>
#include <openvr.h>
#include <stdio.h>
using namespace vr;


class VRPN_API vrpn_Tracker_Vive : public vrpn_Analog,
                                public vrpn_Button_Filter, public vrpn_Tracker {
public:
    vrpn_Tracker_Vive(const char* name, vrpn_Connection* c, int numHMD, int numControllers, int numTrackers);

    ~vrpn_Tracker_Vive();

    /// Called once through each main loop iteration to handle updates.
    virtual void mainloop();

	struct _ControllerData {
        // Fields to be initialzed by iterateAssignIds() and setHands()
        int deviceId = -1;  // Device ID according to the SteamVR system
        int hand = -1;      // 0=invalid 1=left 2=right
        int idtrigger = -1; // Trigger axis id
        int idpad = -1;     // Touchpad axis id

        // Analog button data to be set in ContollerCoods()
        float padX;
        float padY;
        float trigVal;

        // Position set in ControllerCoords()
        HmdVector3_t pos;
        HmdQuaternion_t rot;

        bool isValid;
    };
    typedef struct _ControllerData ControllerData;

	struct _TrackerData {
        int deviceId = -1; // Device ID according to the SteamVR system
        HmdVector3_t pos;
        HmdQuaternion_t rot;
        bool isValid;
    };
    typedef struct _TrackerData TrackerData;

	ControllerData controllers[2];
    TrackerData* trackers;

    int hmdDeviceId = -1;



protected:
    int _numHMD; ///< 1 = track HMD
    int _numControllers;          ///< How many buttons to open
    int _numTrackers;
    struct timeval timestamp; ///< Time of the last report from the device

    virtual void
    clear_values(void); ///< Set all buttons, analogs and encoders back to 0

    /// Try to read reports from the device.  Returns 1 if a complete
    /// report received, 0 otherwise.  Sets status to current mode.
    virtual int get_report(void);

    /// send report iff changed
    virtual void
    report_changes(vrpn_uint32 class_of_service = vrpn_CONNECTION_LOW_LATENCY);

    /// send report whether or not changed
    virtual void
    report(vrpn_uint32 class_of_service = vrpn_CONNECTION_LOW_LATENCY);

    // NOTE:  class_of_service is only applied to vrpn_Analog
    //  values, not vrpn_Button, which are always vrpn_RELIABLE

	/* First decides whether or not to call iterateAssignIds() see the
     * conditions in .cpp Then calls HMDCoords() and ControllerCoords()
     */
    void ParseTrackingFrame();

    /* Takes a VREvent pointer when called by RunProcedure()
     * Switches through the common VREvents
     * If it is a button event (#200-203) then calls dealWithButtonEvent
     * Returns false if the event should cause the application to quit (like
     * SteamVR quitting)
     */
    bool ProcessVREvent(const VREvent_t& event);

    /* First prints the button event data to the terminal
     * Tests to see if it was the ApplicationMenu button that was pressed. If
     so, switch modes
     * Tests that it was a grip press or unpress event and stores the y-bounds
     for the current cylinder if applicable
     * Tests if it was a trigger press or unpress and stroes the x/z bounds for
     the current cylinder if applicable
     * Finally, tests if it was a touchpad press and increments, decrements,
         deletes or rumbles the current cylinder depending on where the press
     was
    */
    void dealWithButtonEvent(VREvent_t);

	/* One of the best methods to look at if just trying to learn how to print
	 tracking data
	 * In only about five lines, gets the HMD pose from the VR system, converts the
	 pose to xyz coordinates, and prints this data to the terminal
	*/
	void HMDCoords();

    /* For each controller:
     * Gets controller state and pose from the VR system
     * Prints controller coords
     * Gets/prints button states (analog data) from the conrollers
     * Rumbles the controllers based on the current mode/inputs
     */
    void ControllerCoords();

    /* CURRENTLY NOT BEING CALLED BY ANY FUNCTION
     * If called every frame before ControllerCoords, should result in smoother
         contoller reconnection after leaving room bounds
    */
    void setHands();

    void TrackerCoords();

	/* Iterates across all the possible device ids (I think the number is 16 for
	 the Vive)
	 * For each possible id, tests to see if it is the HMD or a controller
	 * If it is the HMD, assign hmdDeviceId to the HMD's id
	 * If it is a controller, assign one of the controller structs' deviceId to the
	 id then set handedness and the axis ids of the controller.
	*/
    void iterateAssignIds();



private:
    // The pointer to the VR system for VR function calls. Initialized in the
    // constructor.
    IVRSystem* vr_pointer = NULL;

    int buttonMapping(int viveButton);
    const int BUTTONS_PER_CONTROLLER = 4;
    const int ANALOGS_PER_CONTROLLER = 3;



    int vive2vprn_body(int id, HmdMatrix34_t matrix, struct timeval timestamp);

    // Returns xyz coordinates from the matrix that is returned by the VR system
    // functions
    // see the HMDCoords() and ControllerCoords() methods for usage

    HmdVector3_t GetPosition(HmdMatrix34_t matrix);

	HmdVector3_t GetEulerRotation(HmdMatrix34_t matrix);

    HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);

    HmdQuaternion_t ProcessRotation(HmdQuaternion_t quat);
};

#endif
