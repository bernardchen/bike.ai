//
//  ContentView.swift
//  EE149-Final-Project
//
//  Created by ananya mukerjee on 12/6/19.
//  Copyright © 2019 Bike.ai. All rights reserved.
//


import SwiftUI
import UIKit


struct ContentView: View {
    var body: some View {
        CustomController ()
        
    }
}

#if DEBUG
struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
#endif
 
struct CustomController: UIViewControllerRepresentable{
    func updateUIViewController(_ uiViewController: UIViewController, context: UIViewControllerRepresentableContext<CustomController>) {
    }
    
    
    typealias UIViewControllerType = UIViewController
        
    
    
    func makeUIViewController(context: UIViewControllerRepresentableContext<CustomController>) -> CustomController.UIViewControllerType {
        let storyboard = UIStoryboard(name: "LaunchScreen", bundle: Bundle.main)
        let controller = storyboard.instantiateViewController(identifier: "Home")
        return controller
    }
}

