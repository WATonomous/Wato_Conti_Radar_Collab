from rosbag import Bag
import argparse
import os, os.path

def rename_topic(filename, oldtopic, newtopic, newfilename):
	with Bag(newfilename, 'w') as outbag:
		print ("Converting {} to {}".format(oldtopic, newtopic))
		topics = []
		converted = 0
		for topic, msg, time in Bag(filename).read_messages():
			if not topic in topics:
				topics.append(topic)
			if topic == oldtopic:
				converted = 1
				outbag.write(newtopic, msg, time)
			else:
				outbag.write(topic, msg, time)
		print ("Topics in this rosbag: ")
		for x in topics:
			print(topics)
		print("Saved to: {}".format(newfilename))
		if converted == 0:
			print ("WARNING: ROSBAG DID NOT CONTAIN {}".format(oldtopic))
		else:
			print ("SUCCESS")

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('--filename', required=True, metavar='"Rosbag path"', type=str, help='pathtofile.bag')
	parser.add_argument('--oldtopic', required=True, metavar='"Name of old topic"', type=str, help="/nameoftopic")
	parser.add_argument('--newtopic', required=False, default='/unfiltered_radar_packet', metavar='"Name of new topic"', type=str, help='/nameoftopic, defaults to /unfiltered_radar_packet_1')
	parser.add_argument('--newfilename', required=False, metavar='"New rosbag path"', type=str, help='newpathtobag.bag, defaults to filtered$CURRENTNAME$')
	args = parser.parse_args()
	if args.newfilename is None: 
		in_basename = os.path.splitext(os.path.basename(os.path.expanduser(args.filename)))[0]
		args.newfilename = os.path.join(os.path.expanduser(os.path.dirname(args.filename)), 'filtered_' + in_basename + '.bag')
	rename_topic(os.path.expanduser(args.filename), args.oldtopic, args.newtopic, args.newfilename)

if __name__ == "__main__":
	main()
	
