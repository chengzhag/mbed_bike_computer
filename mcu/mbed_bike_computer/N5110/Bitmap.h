#ifndef BITMAP_H
#define BITMAP_H

#include <vector>

// Forward declarations
class N5110;

/**
 * @brief  A black & white bitmap that can be rendered on an N5110 screen
 * @author Alex Valavanis <a.valavanis@leeds.ac.uk>
 * 
 * @code
  // First declare the pixel map data using '1' for black,
  // or '0' for white pixels
  static int sprite_data[] = {
    0,0,1,0,0,
    0,1,1,1,0,
    0,0,1,0,0,
    0,1,1,1,0,
    1,1,1,1,1,
    1,1,1,1,1,
    1,1,0,1,1,
    1,1,0,1,1
  };

  // Instantiate the Bitmap object using the data above
  Bitmap sprite(sprite_data, 8, 5); // Specify rows and columns in sprite
  
  // We can render the bitmap wherever we want on the screen
  sprite.render(lcd, 20, 6); // x and y locations for rendering
  sprite.render(lcd, 30, 10);
  
  // We can also print its values to the terminal
  sprite.print();
 * @endcode
 */
class Bitmap
{
private:
    /**
     * @brief The contents of the drawing, with pixels stored in row-major order
     * @details '1' represents a black pixel; '0' represents white
     */
    std::vector<int> _contents;
    
    unsigned int _height; ///< The height of the drawing in pixels
    unsigned int _width;  ///< The width of the drawing in pixels
    
public:
    Bitmap(int const          *contents,
           unsigned int const  height,
           unsigned int const  width);

    int get_pixel(unsigned int const row,
                  unsigned int const column) const;

    void print() const;

    void render(N5110 &lcd,
                unsigned int const x0,
                unsigned int const y0) const;
};

#endif // BITMAP_H