#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <cairo.h>
#include <cairo-xlib.h>
//#include <glib/gprintf.h>
#include <rsvg.h>
//#include <rsvg-cairo.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "lidar.h"
#include "base_module.h"
#include "gui.h"
#include "mcl.h"

#define COMPASS_RADIUS 70

#define DISPLAY_FREQUENCY 10
#define CROP_DISTANCE (6000 * 4)
#define ENLARGE_CENTER 500

#define LOOP_DELAY 30000

#define RIGHT_ARROW 0xff53
#define LEFT_ARROW 0xff51
#define UP_ARROW 0xff52
#define DOWN_ARROW 0xff54
#define ESC_KEY 0xff1b
#define LEFT_MOUSE_BUTTON -1

#define RAY_USUAL_TYPE 1
#define RAY_AZIMUTH_TYPE 2
#define RAY_ZERO_TYPE 3

#define MAP_ZOOM_FACTOR_HYPO 1
#define MAP_ZOOM_FACTOR 1//(MAP_ZOOM_FACTOR_HYPO * 40)
#define MAP_XOFFSET 0 // -120
#define MAP_YOFFSET 0 // -270

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

const int minx = 1, miny = 1, maxx = 8, maxy = 8;

static lidar_data_type lidar_data;

static hypo_t hypo[HYPO_COUNT];

static double guiWidth = 600;
static double guiHeight = 600;

void draw_ray(int i, int ray_type)
{
   int x, y, center_x, center_y;
   if (ray_type == RAY_USUAL_TYPE)
   {
      x = (int)((ENLARGE_CENTER + lidar_data.distance[i]) / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      y = (int)((ENLARGE_CENTER + lidar_data.distance[i]) / 32000.0 * guiHeight * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      center_y = (int)(ENLARGE_CENTER / 32000.0 * guiHeight * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.1, 0.1, 0.8);
   } else if (ray_type == RAY_AZIMUTH_TYPE)
   {
      x = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * sin(M_PI * i / 64.0 / 180.0) + guiWidth / 2);
      y = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * cos(M_PI * i / 64.0 / 180.0) + guiHeight / 2);
      center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * i / 64.0 / 180.0) + guiWidth / 2);
      center_y = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * cos(M_PI * i / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.8, 0.8, 0.3);
   } else if (ray_type == RAY_ZERO_TYPE)
   {
      x = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      y = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      center_y = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.8, 0.8, 0.8);
   }
   
   cairo_move_to(gui, x, guiHeight - y);
   cairo_line_to(gui, center_x, guiHeight - center_y);
   cairo_stroke(gui);
   cairo_set_source_rgb(gui, 1, 0.3, 0.3);
   cairo_arc(gui, x, guiHeight - y, 2, 0, 2 * M_PI);
   cairo_stroke(gui);
}

void *gui_thread(void *arg)
{
   double cpWidth = 350;
   double cpHeight = 200;
   base_data_type base_data;
   int disp_counter = 0;
   GError *err;
   RsvgHandle *svg_handle = rsvg_handle_new_from_file ("mapa_pavilonu_I.svg", &err);
   if (svg_handle == 0)
   {
      mikes_log(ML_ERR, "could not load svg file");
      mikes_log(ML_ERR, err->message);
   }
   
   int intersected = 0;
   
   while (program_runs)
   {
      disp_counter++;
      if (disp_counter == DISPLAY_FREQUENCY)
      {
         disp_counter = 0;
         get_base_data(&base_data);
         get_lidar_data(&lidar_data);
         
         /* lidar window */
         cairo_push_group(gui);
         cairo_set_source_rgb(gui, 1, 1, 1);
         cairo_paint(gui);
         cairo_set_line_width(gui, 2);
         
         for (int i = 0; i < lidar_data.count; i++)
         {
            if (lidar_data.angle[i] > (120 * 64) && lidar_data.angle[i] < (240 * 64)) continue;
            if (lidar_data.distance[i] == 0) {
               draw_ray(i, RAY_ZERO_TYPE);
               continue;
            }
            if (lidar_data.distance[i] > CROP_DISTANCE) lidar_data.distance[i] = CROP_DISTANCE;
            draw_ray(i, RAY_USUAL_TYPE);
         }
         
         if (get_current_azimuth() != NO_AZIMUTH)
            draw_ray(get_current_azimuth() - base_data.heading, RAY_AZIMUTH_TYPE);
         
         cairo_pop_group_to_source(gui);
         cairo_paint(gui);
         cairo_surface_flush(gui_surface);
         //gui_cairo_check_event(gui_surface, 0);
         
         /* compass window */
         cairo_push_group(cp_gui);
         cairo_set_source_rgb(cp_gui, 1, 1, 1);
         cairo_paint(cp_gui);
         cairo_set_line_width(cp_gui, 3);
         cairo_set_source_rgb(cp_gui, 0.1, 0.3, 1);
         cairo_arc(cp_gui, cpWidth / 2, cpHeight / 2, COMPASS_RADIUS + 5, 0, 2 * M_PI);
         cairo_stroke(cp_gui);
         double compass_x = COMPASS_RADIUS * sin(M_PI * base_data.heading / 180.0);
         double compass_y = COMPASS_RADIUS * cos(M_PI * base_data.heading / 180.0);
         cairo_set_source_rgb(cp_gui, 1, 0, 0.2);
         cairo_move_to(cp_gui, cpWidth / 2 + compass_x, cpHeight / 2 - compass_y);
         cairo_line_to(cp_gui, cpWidth / 2, cpHeight / 2);
         cairo_stroke(cp_gui);
         cairo_pop_group_to_source(cp_gui);
         cairo_paint(cp_gui);
         cairo_surface_flush(cp_gui_surface);
         
         /* map window */
         cairo_push_group(map_gui);
         cairo_set_source_rgb(map_gui, 1, 1, 1);
         cairo_paint(map_gui);
         cairo_set_source_rgb(map_gui, 0.1, 0.3, 1);
         
         rsvg_handle_render_cairo(svg_handle, map_gui);
         
         get_mcl_data(hypo);
         for (int i = 0; i < HYPO_COUNT; i++)
         {
            double x = hypo[i].x;
            double y = hypo[i].y;
            double alpha = hypo[i].alpha;
            double w = hypo[i].w;
            
            cairo_set_source_rgb(map_gui, 1 - w*0.8 - 0.1, 1 - w*0.8 - 0.1, 1 - w*0.8 - 0.1);
            cairo_set_line_width(map_gui, 3);
            
            cairo_set_line_width(map_gui, 10);
            double hypoax = 5*cos(M_PI * alpha / 180);
            double hypoay = -5*sin(M_PI * alpha / 180);
            
            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);
            
            hypoax = 3*cos(M_PI * alpha / 180 + M_PI/2);
            hypoay = -3*sin(M_PI * alpha / 180 + M_PI/2);
            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);
            
            hypoax = 3*cos(M_PI * alpha / 180-M_PI/2);
            hypoay = -3*sin(M_PI * alpha / 180- M_PI/2);
            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);
            
         }

         
         for (int i = 0; i < 1; ++i) {
            double myx = hypo[i].x;
            double myy = hypo[i].y;
            double myangle = hypo[i].alpha;

//            double myw = hypo[i].w;
            
            // TODO pocitanie sin a cos
            double mycos = cos(myangle * M_PI / 180.0);
            double mysin = sin(myangle * M_PI / 180.0);

            
            double dist = line_inter_poly_i(myx, myy, myy + mycos, myy - mysin, 0);
            
            cairo_set_source_rgb(map_gui, 1, 0.1, 0.1);
            cairo_set_line_width(map_gui, 10);
            
            cairo_move_to(map_gui, (int) myx, (int) myy);
            cairo_line_to(map_gui, (int) (myx + (mycos*dist)), (int) (myy - (mysin*dist)));
            
            cairo_stroke(map_gui);

            cairo_set_source_rgb(map_gui, 0.1, 1, 0.1);
            cairo_move_to(map_gui, (int) myx, (int) myy);
            cairo_line_to(map_gui, (int) (myx + (mycos*100)), (int) (myy - (mysin*100)));
            
            cairo_stroke(map_gui);
            
            if (intersected ==0) {
               mikes_log_val(ML_DEBUG, "hypo alpha", (int) myangle);
               mikes_log_double2(ML_DEBUG, "hypo point", myx, myy);
               mikes_log_double2(ML_DEBUG, "hypo cos sin", mycos, mysin);
               mikes_log_val(ML_DEBUG, "hypo dist = ", (int) dist);
            }

         }
         intersected = 1;

         
         
         cairo_pop_group_to_source(map_gui);
         cairo_paint(map_gui);
         cairo_surface_flush(map_gui_surface);
      }
      
      int event = gui_cairo_check_event(cp_gui_surface, 0);
      if ((event != 0) && (!start_automatically))
         start_automatically = 1;
      switch (event)
      {
         case RIGHT_ARROW:
            user_dir = USER_DIR_RIGHT;
            user_control = 1;
            break;
            
         case LEFT_ARROW:
            user_dir = USER_DIR_LEFT;
            user_control = 1;
            break;
            
         case DOWN_ARROW:
            user_dir = USER_DIR_BACK;
            user_control = 1;
            break;
            
         case UP_ARROW:
            user_dir = USER_DIR_ONOFF;
            user_control = 1;
            break;
            
         case ESC_KEY:
            program_runs = 0;
            mikes_log(ML_INFO, "quit by ESC");
            break;
            
         case 32:
            user_dir = USER_DIR_BACKUP;
            user_control = 1;
            break;
            
         case LEFT_MOUSE_BUTTON:
            user_control = 1 - user_control;
            break;
            
            //default:
            //   printf("event %d\n", event);
      }
      
      usleep(LOOP_DELAY);
   }
   
   gui_shutdown();
   mikes_log(ML_INFO, "gui quits.");
   threads_running_add(-1);
   return 0;
}

