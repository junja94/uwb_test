"""
The Pozyx ready to range tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_range/Python
This demo requires two Pozyx devices. It demonstrates the ranging capabilities and the functionality to
to remotely control a Pozyx device. Move around with the other Pozyx device.
This demo measures the range between the two devices. The closer the devices are to each other, the more LEDs will
light up on both devices.
"""
import numpy as np
import pypozyx
from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
import math

class Ranger(object):
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, anchor_configs, master_id, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION, remote_id=None):
        self.pozyx = pozyx
        self.destination_ids = []
        for anchor in anchor_configs:
            self.destination_ids.append(anchor['id'])

        self.remote_id = remote_id
        self.master_id = master_id
        self.protocol = protocol

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
        print("------------POZYX RANGING V{} -------------".format(version))
        print("NOTES: ")
        print(" - Change the parameters: ")
        print("\tdestination_id(target device)")
        print("\trange_step(mm)")
        print("")

        # if self.remote_id is None:
        #     for device_id in [self.remote_id, *self.destination_ids]:
        #         self.pozyx.printDeviceInfo(device_id)
        # else:
        for device_id in [self.remote_id, *self.destination_ids]:
            if device_id == self.master_id:
                device_id = None
            firmware = pypozyx.SingleRegister()
            status = pozyx.getFirmwareVersion(firmware, device_id)
            print(firmware, ". ", status)

            self.pozyx.printDeviceInfo(device_id)
        # exit()
        print("")
        print("- -----------POZYX RANGING V{} -------------".format(version))
        print("")
        print("START Ranging: ")

        # make sure the local/remote pozyx system has no control over the LEDs.
        # led_config = 0x0
        # self.pozyx.setLedConfig(led_config, self.remote_id)
        # # do the same for the destination.
        # self.pozyx.setLedConfig(led_config, self.destination_id)
        # set the ranging protocol
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

    def loop(self):
        """Performs ranging and sets the LEDs accordingly"""
        device_range = DeviceRange()
        ranges = np.zeros((len(self.destination_ids), 1))
        for i, destination in enumerate(self.destination_ids):
            status = self.pozyx.doRanging(
                destination, device_range, self.remote_id)
            ranges[i] = device_range.distance

            if status != POZYX_SUCCESS:
                error_code = SingleRegister()
                status = self.pozyx.getErrorCode(error_code)
                if status == POZYX_SUCCESS:
                    print("ERROR Ranging, local %s" %
                          self.pozyx.getErrorMessage(error_code))
                else:
                    print("ERROR Ranging, couldn't retrieve local error")
        return ranges


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()


    # remote = False               # whether to use the given remote device for ranging
    # if not remote:
    #     remote_id = None

    x = 300
    y = 550
    anchors = []
    anchors.append({"id": 0x114A,
                    "position": [0, 0, 0]})
    anchors.append({"id": 0x117A,
                    "position": [x, 0, 0]})
    anchors.append({"id": 0x1125,
                    "position": [x, y, 0]})
    anchors.append({"id": 0x1138,
                    "position": [0, y, 0]})


    # the ranging protocol, other one is PozyxConstants.RANGE_PROTOCOL_PRECISION
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION

    pozyx = PozyxSerial(serial_port)

    remote_id = 0x6878        # the network ID of the remote device
    master_id = 0x117A

    for anchor in anchors:
        if anchor['id'] == master_id:
            # anchor['id'] = None
            break
    # remote_id = None

    r = Ranger(pozyx, anchors,master_id, ranging_protocol, remote_id)

    r.setup()

    np.set_printoptions(precision=3)
    np.set_printoptions(suppress=True)
    while True:
        # print("r")
        ranges = r.loop()
        print("test, ", ranges)

        # d1 = 300
        # d2 = 0
        # d3 = 550
        # d4 = 626.5

        num_anchors = len(anchors)

        x = np.array([0.0, 0.0]).transpose()
        estimated_ranges = np.zeros_like(ranges)
        H = np.zeros([num_anchors, 2])

        # iterate until the delta_x becomes small enough
        ii = 0
        while ii < 10:

            i = 0
            for anchor in anchors:
                # calculate the predicted ranges
                print("test ")
                print(anchor["position"])
                print(x)
                estimated_ranges[i] = \
                    math.sqrt((anchor["position"][0] - x[0])**2 + (anchor["position"][1] - x[1])**2)

                # calculate the direction cosines
                H[i][0] = (anchor["position"][0] - x[0]) / (1e-12 + estimated_ranges[i])
                H[i][1] = (anchor["position"][1] - x[1]) / (1e-12 + estimated_ranges[i])
                i += 1

            # observed minus predicted ranges
            delta_roo = ranges - estimated_ranges

            # pseudoinverse
            delta_x = np.linalg.pinv(np.transpose(H) @ H) @ np.transpose(H) @ delta_roo

            # subtract the delta position from the position calculated
            # in previous round
            x = x - delta_x
            # stop iteration when the delta position becomes small enough
            if np.linalg.norm(delta_x) < 0.001:
                break
            ii+=1

        print(x)