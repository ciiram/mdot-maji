import sys
import time
import base64
import ttn
import datetime as dt


from influxdb import InfluxDBClient

app_id = "mdot-range-test"
access_key = "ttn-account-v2.z3SFKHHP1iLuAOSkaslI_FG0tnAMGtqPnaUv9pkZtUo"

GTW_IDS = ['eui-7276ff00080e0675',
           'eui-00800000a0002125',
           'iot-mashinani',
           'eui-0002fcc23d0b760a']         # gateway of interest

db_client = InfluxDBClient(host='localhost', port=8086)
db_client.create_database('ol_pejeta_mdot_range_test')
db_client.switch_database('ol_pejeta_mdot_range_test')

def uplink_callback(msg, client):

  print("Received uplink from ", msg.dev_id)

  for gateway in GTW_IDS:
    gtws_seen = [gtw.gtw_id for gtw in msg.metadata.gateways]

    print(gateway, gtws_seen)
    if gateway in gtws_seen:

      influxdb_entry = {}

      influxdb_entry['time'] = msg.metadata.time
      fields = {}

      fields['data_rate'] = msg.metadata.data_rate
      fields['airtime'] = msg.metadata.airtime
      fields['frequency'] = msg.metadata.frequency
      fields['coding_rate'] = msg.metadata.coding_rate


      for gtw in msg.metadata.gateways:
        if gtw.gtw_id == gateway:
          fields['rssi'] = float(gtw.rssi)
          fields['snr'] = float(gtw.snr)

      try:
        fields['Data'] = float(int.from_bytes(base64.b64decode(msg.payload_raw),
                                              "big"))
      except:
        pass

      influxdb_entry['fields'] = fields
      influxdb_entry['measurement'] = 'Range Test'
      influxdb_entry['tags'] = {'sensor': msg.dev_id,
                                'gateway': gateway}

      print(influxdb_entry)
      print(gateway, db_client.write_points([influxdb_entry]))

handler = ttn.HandlerClient(app_id, access_key)

# using mqtt client
mqtt_client = handler.data()
mqtt_client.set_uplink_callback(uplink_callback)
mqtt_client.connect()

while True:
  try:
    time.sleep(60)
  except KeyboardInterrupt:
    print('Closing ...')
    mqtt_client.close()
    sys.exit(0)
