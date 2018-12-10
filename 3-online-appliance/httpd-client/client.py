from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library
standard_library.install_aliases()
from builtins import str
import http.client
import argparse
import time
import json

def put_handler(ip, espport, nodeport):
    # Establish HTTP connection
    print("Connecting to => " + ip + ":" + espport)
    sess = http.client.HTTPConnection(ip + ":" + espport)
    node = http.client.HTTPConnection(ip + ":" + nodeport)

    headers = {'Content-Type': 'application/json'}
    adc_data = {'adc_reading': '0'}
    hour = -1
    minute = -1

    while True:
        try:
        # Get button state from node server: "1" means LED ON, "0" means OFF
            node.request("GET", url="/ctrl")
            resp = node.getresponse()
            btnState = resp.read()
        # Put current button state to esp session
            sess.request("PUT", url="/ctrl", body=btnState)
            resp = sess.getresponse()
            resp.read()
        # Request light diode ADC data from esp32
            sess.request("GET", url="/adc")
            resp = sess.getresponse()
            adc = resp.read()
        # Send ADC data to node server
            adc_data['adc_reading'] = adc
            adc_json = json.dumps(adc_data)
            node.request("POST", "/adc", adc_json, headers)
            resp = node.getresponse()
            resp.read()
        # Get Time data from node server
            node.request("GET", url="/scheduler")
            resp = node.getresponse()
            scheduler_json = resp.read()
            scheduler_data = json.loads(scheduler_json)
            scheduler_hour = scheduler_data['hour']
            scheduler_minute = scheduler_data['minute']
        # Send Time data to esp32 if schedule time updates
            if (hour != scheduler_hour) or (minute != scheduler_minute):
                hour = scheduler_hour
                minute = scheduler_minute
                scheduler_str = str(scheduler_hour) + ' ' + str(scheduler_minute)
                sess.request("POST", url="/scheduler", body=scheduler_str)
                resp = sess.getresponse()
                resp.read()
        except KeyboardInterrupt:
            print("Quitting . . .")

    # Close HTTP connection
    sess.close()

if __name__ == '__main__':
    # Configure argument parser
    parser = argparse.ArgumentParser(description='Run HTTPd Test')
    parser.add_argument('IP'  , metavar='IP'  ,    type=str, help='Server IP')
    parser.add_argument('espport', metavar='espport',    type=str, help='ESP port')
    parser.add_argument('nodeport', metavar='nodeport',    type=str, help='Node port')
    args = vars(parser.parse_args())

    # Get arguments
    ip   = args['IP']
    espport = args['espport']
    nodeport = args['nodeport']

    # Call PUT handler
    put_handler (ip, espport, nodeport)
