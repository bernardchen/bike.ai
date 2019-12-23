//
//  ViewController.swift
//  EE149BIKE
//
//  Created by ananya mukerjee on 12/7/19.
//  Copyright © 2019 Bike.ai. All rights reserved.
//
import CoreBluetooth
import UIKit

class ViewController: UIViewController {
   var peripheralManager = CBPeripheralManager()
       override func viewDidLoad() {
           super.viewDidLoad()
           peripheralManager = CBPeripheralManager(delegate: self, queue: nil)
    }
           
    func peripheralManagerDidStartAdvertising(error: NSError?)
    {
        if let error = error {
            print("Failed… error: \(error)")
            return
        }
        print("Succeeded!")
    }


}
extension ViewController : CBPeripheralManagerDelegate {
    
       func peripheralManagerDidUpdateState(_ peripheral: CBPeripheralManager) {
        switch peripheral.state {
        case .poweredOff:
            print("No Power")
        case .unknown:
            print("Unknown")
        case .resetting:
            print("Reset")
        case .unsupported:
            print("Unsupported")
        case .unauthorized:
            print("No Auth")
        case .poweredOn:
            print("There is Power")
            let advertisementData = [CBAdvertisementDataLocalNameKey: "TestDevice"]
            //peripheralManager.startAdvertising(advertisementData)
        @unknown default:
            print("Default Case")
        }

    }
}

