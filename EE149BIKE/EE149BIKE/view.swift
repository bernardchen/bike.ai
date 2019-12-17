//
//  view.swift
//  EE149BIKE
//
//  Created by ananya mukerjee on 12/14/19.
//  Copyright © 2019 Bike.ai. All rights reserved.
//

//
//  ViewController.swift
//  EE149BIKE
//
//  Created by ananya mukerjee on 12/7/19.
//  Copyright © 2019 Bike.ai. All rights reserved.
//
import CoreBluetooth
import UIKit

class View: UIViewController,UITextFieldDelegate {
    var brakeColor = 2
    var turnColor = 2
    var autoBrake = 1
    var dist = 0
    
    @IBAction func doneSetting(_ sender: Any) {
        let temp = peripheralManager as! CBPeripheralManager?
        temp?.removeAllServices()
        if temp?.state != .poweredOn {
            return
        } else {
        print("There is Power")
             let temp = peripheralManager as! CBPeripheralManager?
             let advertisementData = [CBAdvertisementDataLocalNameKey:
                  "LIL"]
              let serviceUUID = CBUUID(string: "FFE0")
                               let service = CBMutableService(type: serviceUUID, primary: true)
                               let characteristicUUID = CBUUID(string: "FFE1")
              let properties: CBCharacteristicProperties = [.read]
              let permissions: CBAttributePermissions = [.readable]
              let val1 = String(turnColor,radix: 2)
              let val2 = String(brakeColor,radix: 2)
              let val3 = String(dist,radix: 2)
              let val4 = String(autoBrake,radix: 2)
              let serialized = "\(val1)\(val2)\(val3)\(val4)"
              print(serialized)
                              let characteristic1 = CBMutableCharacteristic(
                                                  type: characteristicUUID,
                                                  properties: properties,
                                                  value: Data(serialized.utf8),
                                                  permissions: permissions)
              let characteristic2 = CBMutableCharacteristic(
              type: characteristicUUID,
              properties: properties,
              value: nil,
              permissions: permissions)
              let characteristic3 = CBMutableCharacteristic(
                       type: characteristicUUID,
                       properties: properties,
                       value: nil,
                       permissions: permissions)
              let characteristic4 = CBMutableCharacteristic(
              type: characteristicUUID,
              properties: properties,
              value: nil,
              permissions: permissions)
              service.characteristics = [characteristic1,characteristic2,characteristic3,characteristic4]
              temp!.add(service)
        temp!.startAdvertising(advertisementData)
            let seconds = 1.0
            DispatchQueue.main.asyncAfter(deadline: .now() + seconds) {
                temp!.stopAdvertising()
            }
        }
    }
    
    @IBAction func autoBrake(_ sender: UISegmentedControl  ) {
        if (sender.selectedSegmentIndex == 0) {
            autoBrake = 0
        } else {
            autoBrake = 1
        }
        print("Toggled Auto Brake")
    }
    @IBAction func yellowBrake(_ sender: Any) {
        print("yellowBrake")
        brakeColor = 2
    }
    @IBAction func redBrake(_ sender: Any) {
        print("redBrake")
        brakeColor = 1
    }
    @IBAction func blueBrake(_ sender: Any) {
        print("greenBrake")
        brakeColor = 0
    }
    @IBAction func blueTurn(_ sender: Any) {
        turnColor = 0
    }
    
    @IBOutlet weak var myTextField: UITextField!
    @IBAction func redTurn(_ sender: Any) {
        turnColor = 1
    }
    @IBAction func yellowTurn(_ sender: Any) {
        turnColor = 2
    }
    var peripheralManager: CBManager?


       override func viewDidLoad() {
           super.viewDidLoad()
        self.myTextField.delegate = self

        peripheralManager = CBPeripheralManager(delegate: self, queue: nil).self
        let temp = peripheralManager as! CBPeripheralManager?
                 
       
    }
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if (textField.text!.count > 1){
            return true
        }
        dist = Int(textField.text!)!
          self.view.endEditing(true)
          return false
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
extension View : CBPeripheralManagerDelegate {
    
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
            print("Power")
        @unknown default:
            print("Default Case")
                
        }

    }
}

