/*
 * usbtiny - An example USBTiny driver for OSX, based on avrdude code
 * Copyright (C) 2007 Limor Fried
 * Copyright (C) 2009 Jon Nall
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <mach/mach.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOCFPlugIn.h>
#import <Foundation/Foundation.h>

// these are specifically assigned to USBtiny,
// if you need your own VID and PIDs you can get them for cheap from
// www.mecanique.co.uk so please don't reuse these. Thanks!
const NSUInteger TINYUSB_VID = 0x8087;
const NSUInteger TINYUSB_PID = 0x07dc;

// Generic requests to the USBtiny
const NSUInteger USBTINY_ECHO = 0;   // echo test
const NSUInteger USBTINY_READ = 1;   // read byte (wIndex:address)
const NSUInteger USBTINY_WRITE = 2;  // write byte (wIndex:address, wValue:value)
const NSUInteger USBTINY_CLR = 3;    // clear bit (wIndex:address, wValue:bitno)
const NSUInteger USBTINY_SET = 4;    // set bit (wIndex:address, wValue:bitno)

// Programming requests
const NSUInteger USBTINY_POWERUP = 5;       // apply power (wValue:SCK-period, wIndex:RESET)
const NSUInteger USBTINY_POWERDOWN = 6;     // remove power from chip
const NSUInteger USBTINY_SPI = 7;           // issue SPI command (wValue:c1c0, wIndex:c3c2)
const NSUInteger USBTINY_POLL_BYTES = 8;    // set poll bytes for write (wValue:p1p2)
const NSUInteger USBTINY_FLASH_READ = 9;    // read flash (wIndex:address)
const NSUInteger USBTINY_FLASH_WRITE = 10;  // write flash (wIndex:address, wValue:timeout)
const NSUInteger USBTINY_EEPROM_READ = 11;  // read eeprom (wIndex:address)
const NSUInteger USBTINY_EEPROM_WRITE = 12; // write eeprom (wIndex:address, wValue:timeout)
const NSUInteger USBTINY_DDRWRITE = 13;     // set port direction
const NSUInteger USBTINY_SPI1 = 14;         // a single SPI command

// Flags to indicate how to set RESET on power up
const NSUInteger RESET_LOW = 0;
const NSUInteger RESET_HIGH = 1;

// The SCK speed can be set by avrdude, to allow programming of slow-clocked parts
const NSUInteger SCK_MIN = 1;      // usec delay (target clock >= = 4; MHz)
const NSUInteger SCK_MAX = 250;    // usec (target clock >= = 16; KHz)
const NSUInteger SCK_DEFAULT = 10; // usec (target clock >= = 0;.4 MHz)

// How much data, max, do we want to send in one USB packet?
const NSUInteger CHUNK_SIZE = 128; // must be power of = 2; less than = 256;

// The default USB Timeout
const NSUInteger USB_TIMEOUT = 500; // msec

const NSUInteger SS_PIN = 4;   // Hijack RESET as SS. RESET is PB4

static int findDevice(IOUSBDeviceInterface187*** dev, NSNumber* vid, NSNumber* pid)
{
    int rc = 0;
    mach_port_t masterPort = 0;
    kern_return_t err = IOMasterPort(MACH_PORT_NULL, &masterPort);
    if(err)
    {
        return -1;
    }
    
    NSMutableDictionary* matchingDict = (NSMutableDictionary*)IOServiceMatching(kIOUSBDeviceClassName);
    if(matchingDict == nil)
    {
        mach_port_deallocate(mach_task_self(), masterPort);
        return -1;
    }
    
    [matchingDict setValue:vid forKey:[NSString stringWithCString:kUSBVendorID encoding:NSASCIIStringEncoding]];
    [matchingDict setValue:pid forKey:[NSString stringWithCString:kUSBProductID encoding:NSASCIIStringEncoding]];
    
    io_iterator_t iterator = 0;
    err = IOServiceGetMatchingServices(masterPort, (CFMutableDictionaryRef)matchingDict, &iterator);
    
    // This adds a reference we must release below
    io_service_t usbDeviceRef = IOIteratorNext(iterator);
    
    if(usbDeviceRef == 0)
    {
        NSLog(@"Couldn't find USB device with %x:%x",
            [vid unsignedIntegerValue], [pid unsignedIntegerValue]);
        rc = -1;
    }
    
    // Now, get the locationID of this device. In order to do this, we need to
    // create an IOUSBDeviceInterface for our device. This will create the
    // necessary connections between our userland application and the kernel
    // object for the USB Device.
    else
    {
        SInt32 score;
        IOCFPlugInInterface** plugInInterface = 0;
        
        err = IOCreatePlugInInterfaceForService(usbDeviceRef,
                                                kIOUSBDeviceUserClientTypeID,
                                                kIOCFPlugInInterfaceID,
                                                &plugInInterface, &score);        
        if (err != kIOReturnSuccess || plugInInterface == 0)
        {
            NSLog(@"IOCreatePlugInInterfaceForService returned 0x%08x.", err);
            rc = -2;
        }
        else
        {
            // Use the plugin interface to retrieve the device interface.
            err = (*plugInInterface)->QueryInterface(plugInInterface,
                                                     CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID),
                                                     (LPVOID*)dev);
 
            // Now done with the plugin interface.
            (*plugInInterface)->Release(plugInInterface);
 
            if (err || dev == NULL)
            {
               NSLog(@"QueryInterface returned %d.", err);
               rc = -3;
            }
        }
    }
    
    IOObjectRelease(usbDeviceRef);
    IOObjectRelease(iterator);
    mach_port_deallocate(mach_task_self(), masterPort);
 
    return rc;
}

static int send_ctrl_msg(IOUSBDeviceInterface** dev, const UInt8 request,
    const UInt16 value, const UInt16 index)
{
    // USBTiny returns 8 bytes back at the most unless 
    // USBTINY_FLASH_READ or USBTINY_EEPROM_READ is the request
    uint8_t buf[8];
    memset(buf, 0, 8);

    IOUSBDevRequest req;
    req.bmRequestType = USBmakebmRequestType(kUSBIn, kUSBVendor, kUSBDevice);
    req.bRequest = request;
    req.wValue = value;
    req.wIndex = index;
    req.wLength = 8;
    req.pData = buf;
    req.wLenDone = 0;


    IOReturn rc = (*dev)->DeviceRequest(dev, &req);

    if(request == USBTINY_ECHO)
    {
        NSLog(@"req 0x%x", req.bmRequestType);
        NSLog(@"0x%x/0x%x/0x%x (0x%x/0x%x/0x%x)", req.bRequest, req.wValue, req.wIndex, request, value, index);
        for(int i = 0; i < req.wLenDone; ++i)
        {
            NSLog(@"buf[%d] = 0x%x", i, buf[i]);
        }
    }

    if(rc != kIOReturnSuccess)
    {
        return -1;
    }

    return req.wLenDone;
}

void dataToDisplay(IOUSBDeviceInterface** dev, NSString* data)
{
    NSLog(@"Display %@", data);
    
    send_ctrl_msg(dev, USBTINY_CLR, SS_PIN, 0); // /SS low

    for(NSUInteger i = 0; i < [data length]; ++i)
    {
        send_ctrl_msg(dev, USBTINY_SPI1, [data characterAtIndex:i], 0); // data
    }
    
    send_ctrl_msg(dev, USBTINY_SET, SS_PIN, 0); // /SS high
}

int main(int argc, char** argv)
{
#if 0
    if(argc != 2)
    {
        NSLog(@"usage: %s <string>", argv[0]);
        return 1; 
    }
#endif
    NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];

    int status = 0;
    IOUSBDeviceInterface187** dev = 0;
    status = findDevice(&dev,
        [NSNumber numberWithUnsignedInt:TINYUSB_VID],
        [NSNumber numberWithUnsignedInt:TINYUSB_PID]);

    if(status == 0)
    {
#if 1
        NSLog(@"Found!");
        if ((*dev)->USBDeviceOpen(dev) == kIOReturnSuccess)
        {
            (*dev)->ResetDevice(dev);
            (*dev)->USBDeviceReEnumerate(dev,0);
            (*dev)->USBDeviceClose(dev);
            NSLog(@"Reset!");
        }
#else
        send_ctrl_msg(dev, USBTINY_POWERUP, USB_TIMEOUT, RESET_LOW);
        sleep(1);
        send_ctrl_msg(dev, USBTINY_POWERUP, USB_TIMEOUT, RESET_HIGH);
        sleep(1);

        // Example of getting data back -- this returns 8 bytes
        send_ctrl_msg(dev, USBTINY_ECHO, 0xADDE, 0xEFBE);
        
        // Set PB4 (RESET) to output for /Slave Select
        // 1011 0001
        //
        send_ctrl_msg(dev, USBTINY_DDRWRITE, 0xB1, 0);
        send_ctrl_msg(dev, USBTINY_SET, SS_PIN, 0); // /SS high (use RESET PB4)
        
        dataToDisplay(dev, [NSString stringWithCString:argv[1] encoding:NSASCIIStringEncoding]);
        send_ctrl_msg(dev, USBTINY_POWERDOWN, 0, 0);
#endif
        
        (*dev)->Release(dev);
    }
    
    [pool drain];
    return 0;
}
