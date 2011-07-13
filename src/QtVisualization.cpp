
#include "QtVisualization.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CameraPinhole.h"

#ifdef HAVE_QT_OPENGL
#if (defined WIN32 || defined _WIN32 || defined WINCE)
#include <windows.h>
#endif
#include <GL/gl.h>
#endif

using cv::Ptr;

namespace OpencvSfM{

  std::vector< Ptr<Visualization> > Visualization::all_windows_;
  DECLARE_MUTEX( Visualization::access_windows );


  Visualization::Visualization(std::string window_name, int flags)
    :my_camera( Ptr<Camera>( new CameraPinhole() ) )
  {
    old_mouse_position_x_ = 0;
    old_mouse_position_y_ = 0;
    my_window_name_ = window_name;
    P_MUTEX( access_windows );
    position_in_list_ = all_windows_.size ();
    Visualization::all_windows_.push_back (
      Ptr<Visualization>( this ) );
    V_MUTEX( access_windows );
    cv::namedWindow( window_name, flags );

#ifdef HAVE_QT_OPENGL
    cvCreateOpenGLCallback("POSIT",draw_opengl,&position_in_list_);
#endif
  }

  Visualization::~Visualization()
  {
    /*
    *Nothing to do because if we are here it's because someone,
    *somewhere has done something like this:
    Visualization::all_windows_[ position_in_list_ ] =
    cv::Ptr<Visualization>( NULL ) ;
    */
    cv::destroyWindow( my_window_name_ );
    //try to reduce the size of the window list:
    P_MUTEX( access_windows );
    unsigned int iter_empty = Visualization::all_windows_.size() - 1;
    while( iter_empty >= 0 && 
      Visualization::all_windows_[iter_empty].empty() )
    {
      Visualization::all_windows_.pop_back();
      iter_empty--;
    }
    V_MUTEX( access_windows );
  }


  void Visualization::mouseCallback(int event, int x, int y,
    int flags, void* param)
  {
    CV_Assert( param != NULL );
    //first get the corresponding window:
    unsigned int window_id = *( (unsigned int*) param);
    Ptr<Visualization> current_window;
    P_MUTEX( access_windows );
    if( window_id < Visualization::all_windows_.size() )
      current_window = all_windows_[window_id];
    V_MUTEX( access_windows );
    if( current_window.empty() )
      CV_Error(CV_StsNullPtr, "NULL window handler");
    double old_x = current_window->old_mouse_position_x_,
      old_y = current_window->old_mouse_position_y_;
    current_window->old_mouse_position_x_ = x;
    current_window->old_mouse_position_y_ = y;
    if( (CV_EVENT_FLAG_LBUTTON & flags) == 0 )
      return;//nothing to do


    double dx = old_x - x, dy = old_y - y;

    //dx will rotate the point of view around Z axis
    //and dy around the X axis.
    current_window->my_camera.rotationAroundZ( 90.0 / dx);
    current_window->my_camera.rotationAroundX( 90.0 / dy);
  }

  Ptr<Visualization> Visualization::getWindowByName(std::string window_name,
    int flags)
  {
    unsigned int iter = 0;
    bool notFound = true;
    Ptr<Visualization> out_val;
    P_MUTEX( access_windows );
    for (iter = 0; iter < all_windows_.size() && notFound; ++iter)
    {
      if ( !all_windows_[iter].empty() &&
        all_windows_[iter]->my_window_name_ == window_name )
      {
        notFound = false;
        out_val = all_windows_[iter];
      }
    }
    V_MUTEX( access_windows );
    if( notFound )
    {
      out_val = Ptr<Visualization>( new Visualization(window_name, flags) );
    }
    return out_val;
  }

  Ptr<Visualization> Visualization::getWindowByIndex(unsigned int window_index)
  {
    if ( window_index < all_windows_.size() )
      return all_windows_[ window_index ];
    else
      return Ptr<Visualization>();
  }

#ifdef HAVE_QT_OPENGL

  void Visualization::draw_opengl(void* param)
  {
    CV_Assert( param != NULL );
    //first get the corresponding window:
    unsigned int window_id = *( (unsigned int*) param);
    Ptr<Visualization> current_window;
    P_MUTEX( access_windows );
    if( window_id < Visualization::all_windows_.size() )
      current_window = all_windows_[window_id];
    V_MUTEX( access_windows );
    if( current_window.empty() )
      CV_Error(CV_StsNullPtr, "NULL window handler");


    glLoadIdentity();
    glScalef( 1.0f, 1.0f, -1.0f);
    //glMultMatrixf( (float*)param );
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable( GL_BLEND );
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    //render geometry
    glDisable(GL_BLEND);
    glDisable( GL_LIGHTING );
  }

#else
  //As we don't have Qt or Opengl, create a visualization
  //with standard features

#endif

}
