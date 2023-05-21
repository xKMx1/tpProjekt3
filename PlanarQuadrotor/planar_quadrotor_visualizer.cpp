#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen                                      #
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)     #
 * 3. Animate proppelers (extra points)
 */

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, f_x, f_y;
    int d_w, d_h, s_w, s_h, d_center_x, d_center_y;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    // drone construction characteristics
    d_w = 400;
    d_h = 60;
    s_w = 20;
    s_h = -80;
    d_center_x = d_w/2;
    d_center_y = d_h/2;

    // new frame coordinates
    f_x = q_x + 640 - (d_w/2);
    f_y = q_y + 360 - (d_h/2);


    SDL_Rect drone_base;
    drone_base.x = f_x;
    drone_base.y = f_y;
    drone_base.w = d_w;
    drone_base.h = d_h;

    SDL_Rect drone_stick_1;
    drone_stick_1.x = f_x + 340;
    drone_stick_1.y = f_y+10;
    drone_stick_1.w = s_w;
    drone_stick_1.h = s_h;

    SDL_Rect drone_stick_2;
    drone_stick_2.x = f_x + d_w - 340 - s_w;
    drone_stick_2.y = f_y+10;
    drone_stick_2.w = s_w;
    drone_stick_2.h = s_h;

    SDL_Rect drone_propeler_1;
    drone_propeler_1.x = f_x - 10;
    drone_propeler_1.y = drone_stick_2.y-70;
    drone_propeler_1.w = 120;
    drone_propeler_1.h = s_h/3;

    SDL_Rect drone_propeler_2;
    drone_propeler_2.x = f_x +d_w-110;
    drone_propeler_2.y = drone_stick_2.y-70;
    drone_propeler_2.w = 120;
    drone_propeler_2.h = s_h/3;

    SDL_Texture* texture = SDL_CreateTexture(gRenderer.get(), SDL_PIXELFORMAT_RGBA8888,
    SDL_TEXTUREACCESS_TARGET, d_w, d_h);

    //drawing base
    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &drone_base, q_theta, NULL, SDL_FLIP_NONE);
    // drawing sticks
    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &drone_stick_1, q_theta, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &drone_stick_2, q_theta, NULL, SDL_FLIP_NONE);
    // drawing propelers
    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &drone_propeler_1, q_theta, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &drone_propeler_2, q_theta, NULL, SDL_FLIP_NONE);
}
