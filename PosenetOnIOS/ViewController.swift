//
//  ViewController.swift
//  DeeplabOnIOS
//
//  Created by Dwayne Forde on 2017-12-23.
//

import UIKit
import SafariServices

class ViewController: UIViewController, NostalgiaCameraDelegate {
    
    @IBOutlet var imgView: UIImageView!
    var camera: NostalgiaCamera!
    
    // Initialize Camera when the view loads
    override func viewDidLoad() {
        super.viewDidLoad()
        camera = NostalgiaCamera(controller: self, andImageView: imgView)
    }
    
    // Start it when it appears
    override func viewDidAppear(_ animated: Bool) {
        camera.start()
    }
    
    // Stop it when it disappears
    override func viewWillDisappear(_ animated: Bool) {
        camera.stop()
    }
    
    // Browse to a URL when a match is found
    func matchedItem() {
        let safariController = SFSafariViewController(url: URL(string: "https://www.visitdublin.com/")!)
        present(safariController, animated: true, completion: nil)
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

}

