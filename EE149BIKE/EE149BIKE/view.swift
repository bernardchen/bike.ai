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
    var brakeColor = "Yellow"
    var turnColor = "Green"
    var autoBrake = "1"
    var dist = "2"
    @IBAction func autoBrake(_ sender: UISegmentedControl  ) {
        if (sender.selectedSegmentIndex == 0) {
            autoBrake = "0"
        } else {
            autoBrake = "1"
        }
        print("Toggled Auto Brake")
    }
    @IBAction func yellowBrake(_ sender: Any) {
        print("yellowBrake")
        brakeColor = "Yellow"
    }
    @IBAction func redBrake(_ sender: Any) {
        print("redBrake")
        brakeColor = "Red"
    }
    @IBAction func blueBrake(_ sender: Any) {
        print("greenBrake")
        brakeColor = "Green"
    }
    @IBAction func blueTurn(_ sender: Any) {
        turnColor = "Green"
    }
    
    @IBOutlet weak var myTextField: UITextField!
    @IBAction func redTurn(_ sender: Any) {
        turnColor = "Red"
    }
    @IBAction func yellowTurn(_ sender: Any) {
        turnColor = "Yellow"
    }
    var peripheralManager: CBManager?
       override func viewDidLoad() {
           super.viewDidLoad()
        self.myTextField.delegate = self

        peripheralManager = CBPeripheralManager(delegate: self, queue: nil).self
       
    }
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if (textField.text!.count > 1){
            return true
        }
        dist = textField.text!
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
            print("There is Power")
            let advertisementData = [CBAdvertisementDataLocalNameKey:
                "LIL\(brakeColor)\(turnColor)\(autoBrake) , \(dist)"]
            let temp = peripheralManager as! CBPeripheralManager?
            temp!.startAdvertising(advertisementData)
        @unknown default:
            print("Default Case")
        }

    }
}

