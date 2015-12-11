#!/usr/bin/python2.7

# Python 2.7 global bootloader that should program any pre-defined device
# follows the protocol of the teensy devices, but with user defined amount of address bytes.

# Imports
import sys,os
import usb1
import time
import Tkconstants, tkFileDialog, ttk
import random
from Tkinter import *
from PIL import Image, ImageTk
from intelhex import IntelHex

#Debugging
Debug = 0

#start Directorys to check add your own here 
directory = [["/run/media/$USER/drive/uCodespace/MPLABXProjects/"],
             ["/run/media/$USER/usbdrive/uCodespace/MPLABXProjects/"],
             ["/run/media/$USER/usbdrive/uCodespace/MPLABXProjects/"]]

#image
global canvas, ImageonCanvas, disp

#Device Definitions
#			Device name, address/command bytes,serial num, startAPP, lenghtAPP, transmit size to ep0, color pic, grey pic
Devices = [["Kon_25k50_dev",4,0x0001, 0x1000, 0x6FFF, 64, "/DeviceImages/PIC25K50/Kon_25k50_dev_3x2.jpg","/DeviceImages/PIC25K50/Kon_25k50_dev_3x2_grey.jpg"],
           ["Unknown yet",  2,0x0000, 0x0000, 0x7E00, 64, "/DeviceImages/PIC25K50/Kon_25k50_dev_3x2.jpg","/DeviceImages/PIC25K50/Kon_25k50_dev_3x2_grey.jpg"]]

# USB Variables
#VENDOR_ID = 0x04d8  #original vid & pid
#PRODUCT_ID = 0x0001
VENDOR_ID = 0x1209
PRODUCT_ID = 0xB007
INTERFACE = 0
DATA_SEND = 0x0200 # Could be 0x302?
KEY_ENDPOINT = 1
REPORT_SIZE = 8
REQUEST = usb1.REQUEST_SET_CONFIGURATION
REQUEST_TYPE = usb1.TYPE_CLASS | usb1.RECIPIENT_INTERFACE
Packet_Retries = 3

# Logging starage
global buf
global Logwin, verbose
buf = "Start of Logging\n"

# Variable to store the firmware to flash
global file, shorthand
file = ''

# Variable to store a full transmission packet list over usb.
global transfer
global Fail_tx

# Variable to store states
global attached
attached = False

# Variables for USB interactions
# initialize some stuff & start searching for Gboot device
global context, handle, serial, ac_bytes, boot_type, tx_size, start_add, length_add 
context = usb1.USBContext()

class Verbose_Log(object):
	def write(self, buff):
		global buf, verbose		
		buf += str(buff)+"\n"
		try: 
			if verbose.winfo_exists():
				verbose.insert(END, buff+"\n")
				Logwin.update_idletasks()
		except Exception as e:
			if(Debug):print e
				

def SearchForBoot(contxt):
	global attached, handle, serial
	loge = Verbose_Log()
	if not attached:
		handle = contxt.openByVendorIDAndProductID(
			VENDOR_ID, PRODUCT_ID,
			skip_on_error=True,
		)
		if handle is None:
			attached = False
			#loge.write("[!] Device not found")
		else:
			if handle.kernelDriverActive(INTERFACE):
				handle.detachKernelDriver(INTERFACE)
			handle.claimInterface(INTERFACE)
			device = handle.getDevice()
			serial = device.getbcdDevice()
			attached = True
			load_device_config()
			loge.write("[+] Device Attached")
	else:
		try:
			handle.kernelDriverActive(INTERFACE)
			#loge.write("[+] Device Attached")
		except usb1.USBError:
			attached = False
			load_device_config()
			loge.write("[!] Device Disconnected")
	#print "sleeping 5 sec from thread %d" % i
	#root.after(5000, SearchForBoot(context))

def load_device_config():
	global attached
	global canvas, ImageonCanvas, disp
	global serial, boot_type, tx_size, start_add, length_add
	if attached:
		for x in range(len(Devices)):
			if Devices[x][2] == serial:
				ac_bytes   = Devices[x][1]
				boot_type  = Devices[x][0]
				tx_size    = Devices[x][5]
				start_add  = Devices[x][3]
				length_add = Devices[x][4]
				oimage= Image.open(os.path.dirname(os.path.abspath(__file__)) + Devices[x][6])
				disp = ImageTk.PhotoImage(oimage)
				canvas.itemconfig(ImageonCanvas, anchor=NW,image=disp)
				print "[+] "+str(Devices[x][0])+" Device Found"
	else:
		for x in range(len(Devices)):
			if Devices[x][2] == serial:
				oimage= Image.open(os.path.dirname(os.path.abspath(__file__)) + Devices[x][7])
				disp = ImageTk.PhotoImage(oimage)
				canvas.itemconfig(ImageonCanvas, anchor=NW,image=disp)
		ac_bytes	= None
		boot_type	= None
		tx_size		= None
		start_add	= None
		length_add  = None

# Subfunctions
def do_everything():
	handle = context.openByVendorIDAndProductID(
		VENDOR_ID, PRODUCT_ID,
		skip_on_error=True,
	)
	if handle is None:
		print "Device not present, or user is not allowed to access device."
	else:
		if handle.kernelDriverActive(INTERFACE):
			handle.detachKernelDriver(INTERFACE)
		handle.claimInterface(INTERFACE)
	# Init
		#data = ''+str(chr(0x00))+str(chr(0x10))+str(chr(0))+str(chr(0))
		data = ''+str(chr(0xff))+str(chr(0xff))+str(chr(0xff))+str(chr(0xff))
		for x in range(0,64):
			data = data+(chr(x))
		#data = data+''.join(map(chr, [0xDE,0xAD]*32))
		if(Debug):print len(data)
		if(Debug):print data.encode("hex")
		if(Debug):
			print handle.controlWrite(
				request_type=REQUEST_TYPE,
				request=REQUEST,value=DATA_SEND,
				index=0x000,data=data,timeout=1000)
		else:
			handle.controlWrite(
				request_type=REQUEST_TYPE,
				request=REQUEST,value=DATA_SEND,
				index=0x000,data=data,timeout=1000)
		#handle.bulkRead(1, 13)
		# sleep for 20ms
		time.sleep(0.02)
	#print handle

def donothing():
	global Logwin, verbose
	Logwin = Toplevel(root)
	scrollbary = Scrollbar(Logwin)
	scrollbary.pack(side=RIGHT, fill=Y)
	#scrollbarx = Scrollbar(Logwin, orient=HORIZONTAL)
	#scrollbarx.pack(side=BOTTOM, fill=Y)
	verbose = Text(Logwin, wrap=WORD, yscrollcommand=scrollbary.set)#, xscrollcommand=scrollbarx.set)
	scrollbary.config(command=verbose.yview)
	verbose.insert(INSERT,buf)
	verbose.see(END)
	verbose.pack()
	#button = Button(filewin, text="Do nothing button")
	#button.pack()

def verbosity():
	global Logwin, verbose
	Logwin = Toplevel(root)
	scrollbary = Scrollbar(Logwin)
	scrollbary.pack(side=RIGHT, fill=Y)
	#scrollbarx = Scrollbar(Logwin, orient=HORIZONTAL)
	#scrollbarx.pack(side=BOTTOM, fill=Y)
	verbose = Text(Logwin, wrap=WORD, yscrollcommand=scrollbary.set)#, xscrollcommand=scrollbarx.set)
	scrollbary.config(command=verbose.yview)
	verbose.insert(INSERT,buf)
	verbose.see(END)
	verbose.pack()
	#button = Button(filewin, text="Do nothing button")
	#button.pack()

def handle_events():
	SearchForBoot(context)
	root.after(400, handle_events)

def load_ihex():
	global file, shorthand
	Directory_location = -1
	for x in range(len(directory)):
		if os.path.exists(str(directory[x]).strip("[']")):
			Directory_location = x
			break
	try:
		file = tkFileDialog.askopenfile(parent=root,mode='rb',initialdir=directory[Directory_location],title='Choose an IHEX file',filetypes=[("hex",".hex")])
		shorthand.set(os.path.basename(file.name))
	except:
		file= None

def program_prepare():
	global file, transfer, attached
	global serial, boot_type, tx_size, start_add, length_add
	ih = IntelHex(file)
	passes = False
	addresses = []
	sectors= []
	sector = 0
	sector_write = False
	count  = 0
	lastadd= 0
	if tx_size:
		if file:
			print "[+] device connected and hex loaded"
			# Initiate the sectors list as a tempororary hold containing all possible sectors to be sent
			for sect in range((length_add/tx_size)+1):
				sectors.append([])
			if(Debug):print "sectors="+str(len(sectors))
			if(Debug):print sectors
			if(Debug):print str(len(ih.addresses())) 
			if(Debug):print length_add
			# Truncate hex file to only allowed addresses for the device
			for add in ih.addresses():
				if add >= start_add and add <= (length_add+start_add):
					addresses.append(add)
			print "[+] "+str(len(addresses))+" addresses of valid code" 

			#Start of iteration over valid adressess
			for addr in addresses:
				if addr == (lastadd+1): # If address is sequential to previous one
					if count == 0:
						sectors[sector] = get_sector_add(sector)
						sector_write=True
					sectors[sector].append(chr(ih[addr]))
					#print sector
					count +=1
					lastadd = addr
					#print "prev"
				else:                   # If address is unexpected
					if sector_write:
						for i in range(count,tx_size,1):
							sectors[sector].append(chr(0xFF))
							count +=1
					tmpsector= (addr-start_add)/tx_size
					tmpcount = int(((float(addr-start_add)/tx_size)-((addr-start_add)/tx_size))*tx_size)
					if len(sectors[tmpsector]) == 0:
						sectors[tmpsector] = get_sector_add(tmpsector)
					for i in range((len(sectors[tmpsector])-4), tmpcount-1,1):
						sectors[tmpsector].append(chr(0xFF))
					sectors[tmpsector].append(chr(ih[addr]))
					sector_write = True
					sector = tmpsector
					count  = tmpcount+1
					lastadd= addr
				if count >= tx_size:
					count  = 0
					sector +=1
					sector_write=False
			if sector_write:
				for i in range(count,tx_size,1):
					sectors[sector].append(chr(0xFF))
					count +=1
				sector_write=False
			transfer = []
			for sects in sectors:
				if len(sects)-4 == tx_size:
					transfer.append(sects)
				#else:
					#print str(len(sects))
					#print sects
	
			# Convert transfer to string bytes
			#print type(transfer[2][30])
			for data in range(len(transfer)):
				#print transfer[data]
				tmp = ''.join(transfer[data])
				transfer[data] = tmp
			#print transfer 
			program()
	#print "min addr="+str(ih.minaddr())
	#print "max addr="+str(ih.maxaddr())
	#print ih[0]
	#print len(ih.addresses())
	#print file

def program():
	global transfer, progressbar, Packt_Retries
	Fail_tx = 0
	passes = 0 
	progress_step =(float(100)/len(transfer))
	if(Debug):print progress_step
	if transfer:
		for data in transfer:
			#print data
			for x in range (Packet_Retries):
				try:
					handle.controlWrite(
						request_type=REQUEST_TYPE,
						request=REQUEST,value=DATA_SEND,
						index=0x000,data=data,timeout=1000)
					passes = 1
					x = Packet_Retries
				except:
					passes = 0
					Fail_tx += 1
			progressbar.step(amount=progress_step)
			root.update_idletasks()
		progressbar.stop()
		print "[!]Failed Transfers = "+str(Fail_tx)
		file.close()
		restart()
		exit()

def restart():
	if(Debug):
		print handle.controlWrite(
			request_type=REQUEST_TYPE,
			request=REQUEST,value=DATA_SEND,
			index=0x000,data=''.join([chr(0xff),chr(0xff),chr(0xff),chr(0xff)]),timeout=1000)
	else:
		handle.controlWrite(
		request_type=REQUEST_TYPE,
		request=REQUEST,value=DATA_SEND,
		index=0x000,data=''.join([chr(0xff),chr(0xff),chr(0xff),chr(0xff)]),timeout=1000)

def get_sector_add(sec):
	global serial, boot_type, tx_size, start_add, length_add
	add_sector= ((sec*tx_size)+start_add)
	add_sector_low = add_sector&255
	add_sector_hi  = (add_sector&65280)>>8
	add_sector_upl = (add_sector&16711680)>>16
	add_sector_uph = (add_sector&4278190080)>>24
	return [chr(add_sector_low),chr(add_sector_hi),chr(add_sector_upl),chr(add_sector_uph)]

def check_args():
	global directory, file, shorthand
	if (len(sys.argv) > 1):
		if(Debug):print sys.argv
		if(os.path.isfile(str(sys.argv[1]).strip("[']"))): 
			file = open((str(sys.argv[1]).strip("[']")),'r')
			shorthand.set(os.path.basename(file.name))
			if(Debug):print file
		if(os.path.isdir(str(sys.argv[1]).strip("[']"))):
			directory[0] = str(sys.argv[1]).strip("[']")

def Gen_GUI():
	global shorthand, canvas, disp, ImageonCanvas, progressbar
	
	# Check if File or directory passed
	check_args() 
	
	# Generate Menu
	menubar  = Menu(root)
	filemenu = Menu(menubar, tearoff=0)
	filemenu.add_command(label="Open IHEX", command=load_ihex)
	filemenu.add_separator()
	filemenu.add_command(label="Exit", command=root.quit)
	menubar.add_cascade(label="File", menu=filemenu)
	actionmenu = Menu(menubar, tearoff=0)
	actionmenu.add_command(label="Program", command=program_prepare)
	actionmenu.add_command(label="Restart", command=restart)
	actionmenu.add_separator()
	actionmenu.add_checkbutton(label="Automagic", command=donothing)
	menubar.add_cascade(label="Action", menu=actionmenu)
	helpmenu = Menu(menubar, tearoff=0)
	helpmenu.add_command(label="Verbose Log", command=verbosity)
	helpmenu.add_command(label="About This", command=donothing)
	menubar.add_cascade(label="Help", menu=helpmenu)
	root.config(menu=menubar)

	# Place Quick Action Buttons


	# Place progress bar
	progressbar= ttk.Progressbar(orient=HORIZONTAL, length=300, maximum='100', mode='determinate')
	progressbar.pack()

	# Place Image Box & Initial Image
	canvas= Canvas(root, width=300, height=200)
	bootpath= os.path.dirname(os.path.abspath(__file__)) + "/Boots/"
	if(Debug):print len([name for name in os.listdir(str(bootpath)) if os.path.isfile(str(bootpath)+name)])
	oimage= Image.open(bootpath+"/boot"+str(random.randint(1,len([name for name in os.listdir(str(bootpath)) if os.path.isfile(str(bootpath)+name)])))+".jpg")
	disp = ImageTk.PhotoImage(oimage)
	ImageonCanvas = canvas.create_image(0,0,anchor=NW,image=disp)
	canvas.pack()

	# Place loaded Hex name underneath
	FileLabel = Label(root, textvariable=shorthand)
	FileLabel.pack()


root     = Tk()
shorthand= StringVar()
root.title('Global Boot')
Gen_GUI()
root.after(400, handle_events)
root.mainloop()
