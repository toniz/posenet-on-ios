//
//  NostalgiaCamera.h
//  PosenetOnIOS
//
//  Created by Dwayne Forde on 2017-12-23.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

// Protocol for callback action
@protocol NostalgiaCameraDelegate <NSObject>

- (void)matchedItem;

@end

// Public interface for camera. ViewController only needs to init, start and stop.
@interface NostalgiaCamera : NSObject

-(id) initWithController: (UIViewController<NostalgiaCameraDelegate>*)c andImageView: (UIImageView*)iv;
-(void)start;
-(void)stop;

@end
