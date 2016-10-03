//
// Created by volundr on 8/19/16.
//

#include <SDL_events.h>
#include <SDL_timer.h>
#include "Controls.h"

#define KEY_MOVE_SENSITIVITY 100.f
#define KEY_ROTATE_SENSITIVITY 0.01f
#define MOUSE_SENSITIVITY_X -0.006f
#define MOUSE_SENSITIVITY_Y 0.006f
#define MOUSE_SENSITIVITY_WHEEL -800.f
#define MOUSE_SENSITIVITY_PAN 10.f

namespace RealTimeLidar {

    bool Controls::init() {
        previousTime = SDL_GetTicks();
        return true;
    }

    int Controls::pollEvents(PacketReceiver& receiver, Camera& camera, GridDrawer<CartesianPoint>& gridDrawer) {
        /**************************** HANDLE EVENTS *********************************/
        SDL_Event event;
        while (SDL_PollEvent(&event)) { // process all accumulated events
            switch(event.type) {
                case SDL_MOUSEMOTION:
                    if (event.button.button & SDL_BUTTON_RMASK) {
                        camera.rotatePhi(event.motion.yrel * MOUSE_SENSITIVITY_Y);
                        camera.rotateTheta(event.motion.xrel * MOUSE_SENSITIVITY_X);
                    } else if (event.button.button & SDL_BUTTON_LMASK) {
                        camera.moveBackward(-event.motion.yrel * MOUSE_SENSITIVITY_PAN * camera.lookingUpOrDown());
                        camera.moveLeft(event.motion.xrel * MOUSE_SENSITIVITY_PAN);
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT && !(event.button.button & SDL_BUTTON_RMASK)) {

                    }
                    else if (event.button.button == SDL_BUTTON_RIGHT ) {

                    }
                    break;
                case SDL_MOUSEWHEEL:
                    camera.zoom(event.wheel.y * MOUSE_SENSITIVITY_WHEEL);
                    break;
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_ESCAPE:
                            return 1;
                        case SDLK_F12:
                            gridDrawer.takeScreenShot();
                        default:
                            break;
                    }
                    break;
                case SDL_QUIT:
                    std::cout << "Received Quit Event.\n";
                    return 1;
                default:
                    break;
            }
        }
        return 0;
    }

    void Controls::handleKeyState(PacketReceiver& receiver, Camera& camera) {
        // Determine the time step since last time
        currentTime = SDL_GetTicks();
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;
        // Get current keyboard state ( this is used for smooth controls rather than key press event controls above )
        const Uint8* keyStates = SDL_GetKeyboardState(NULL);
        float keyRotate = KEY_ROTATE_SENSITIVITY * deltaTime;
        float keyMove = KEY_MOVE_SENSITIVITY * deltaTime;
        if (keyStates[SDL_SCANCODE_W]) {
            camera.moveBackward(-keyMove);
        }
        if (keyStates[SDL_SCANCODE_S]) {
            camera.moveBackward(keyMove);
        }
        if (keyStates[SDL_SCANCODE_A]) {
            camera.moveLeft(keyMove);
        }
        if (keyStates[SDL_SCANCODE_D]) {
            camera.moveLeft(-keyMove);
        }
        if (keyStates[SDL_SCANCODE_I]) {
            receiver.increaseNumPacketsToRead(1);
        }
    }

    int Controls::update(PacketReceiver& receiver, Camera& camera, GridDrawer<CartesianPoint>& gridDrawer) {
        int shouldQuit = pollEvents(receiver, camera, gridDrawer);
        handleKeyState(receiver, camera);
        return shouldQuit;
    }
}
