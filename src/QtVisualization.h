
#ifndef _GSOC_SFM_QT_VISUALIZATION_H
#define _GSOC_SFM_QT_VISUALIZATION_H

//Two version of visualization: with / without Qt
#include "config.h" //SFM_EXPORTS

#include <vector>
#include <string>

#include "PointOfView.h"

namespace OpencvSfM{

  
  /*! \brief This class can be used to show images, sequences or 3D object.
  *   
  * When user as computed a structure from a motion, it's often important
  * to see the result of triangulation and the positions of various cameras.
  * This class aims to help user to quickly draw results and interact.
  * If user has Qt and OpenGL, he will be able to move around objects!
  */
  class Visualization
  {
    static std::vector< cv::Ptr<Visualization> > all_windows_;
    CREATE_STATIC_MUTEX( access_windows );

    static void mouseCallback(int event, int x, int y,
      int flags, void* param);

  protected:
#ifdef HAVE_QT_OPENGL
    static void draw_opengl(void* param);
#endif
    std::string my_window_name_;///<Name of this window.
    /**
    * index of this object in the vector all_windows_
    **/
    unsigned int position_in_list_;
    /**
    * Previous mouse positions
    **/
    int old_mouse_position_x_,
      old_mouse_position_y_;

    PointOfView my_camera;///<The position of the user point of view

    /**
    * Can't create a new window!
    * You should use getWindowByName to create a new one!
    * @param window_name name of this window
    **/
    Visualization(std::string window_name, int flags=CV_WINDOW_AUTOSIZE);
  public:
    ~Visualization();
    /**
    * Create a new window or return the previously created window
    * @param window_name name of the wanted window
    * @return Pointer on the window
    **/
    static cv::Ptr<Visualization> getWindowByName(std::string window_name,
      int flags=CV_WINDOW_AUTOSIZE);
    /**
    * return the previously created window using the window's index
    * @param window_index
    * @return Pointer on the window
    **/
    static cv::Ptr<Visualization> getWindowByIndex(unsigned int window_index);
    /**
    * return the index of this window
    * @return index of this window
    **/
    inline unsigned int getIndex(){ return position_in_list_; };

  };

}

#endif