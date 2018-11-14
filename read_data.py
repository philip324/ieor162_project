import numpy as np
import os
import pandas as pd
import IPython


data_dir = os.path.join(os.getcwd(), 'data')
if not os.path.exists(data_dir):
	os.makedirs(data_dir)
input_filename = 'Input_Cost%2C+Location.xlsx'
shipment_req_filename = 'Problem_VehicleShipmentRequirement.csv'


def main():
	input_file = os.path.join(data_dir, input_filename)
	shipment_req_file = os.path.join(data_dir, shipment_req_filename)
	try:
		shipmentReq = pd.read_csv(shipment_req_file)
		input_data = pd.ExcelFile(input_file)
	except:
		print('Put Input_Cost%2C+Location.xlsx and Problem_VehicleShipmentRequirement.csv in ./data')
		return

	LocationLatLong = input_data.parse('LocationLatLong')
	ExistingVDC 	= input_data.parse('ExistingVDC')
	VDCcost 		= input_data.parse('VDCcost')
	LogisticsCost 	= input_data.parse('LogisticsCost')
	print(LocationLatLong.columns)
	# IPython.embed()


if __name__ == '__main__':
	main()