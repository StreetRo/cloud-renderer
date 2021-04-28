#include "clouds.h"

/* perlion noise source: https://en.wikipedia.org/wiki/Perlin_noise */

/* Function to linearly interpolate between a0 and a1
 * Weight w should be in the range [0.0, 1.0]
 */
float interpolate(float a0, float a1, float w) {
    /* // You may want clamping by inserting:
     * if (0.0 > w) return a0;
     * if (1.0 < w) return a1;
     */
    return (a1 - a0) * w + a0;
    /* // Use this cubic interpolation [[Smoothstep]] instead, for a smooth appearance:
     * return (a1 - a0) * (3.0 - w * 2.0) * w * w + a0;
     *
     * // Use [[Smootherstep]] for an even smoother result with a second derivative equal to zero on boundaries:
     * return (a1 - a0) * ((w * (w * 6.0 - 15.0) + 10.0) * w * w * w) + a0;
     */
}

/* Create random direction vector
 */
Vector2f randomGradient(int ix, int iy) {
    // Random float. No precomputed gradients mean this works for any number of grid coordinates
    float random = 2920.f * sin(ix * 21942.f + iy * 171324.f + 8912.f) * cos(ix * 23157.f * iy * 217832.f + 9758.f);
    return Vector2f( cos(random), sin(random) );
}

// Computes the dot product of the distance and gradient vectors.
float dotGridGradient(int ix, int iy, float x, float y) {
    // Get gradient from integer coordinates
    Vector2f gradient = randomGradient(ix, iy);

    // Compute the distance vector
    float dx = x - (float)ix;
    float dy = y - (float)iy;

    // Compute the dot-product
    return (dx*gradient.x() + dy*gradient.y());
}

// Compute Perlin noise at coordinates x, y
float perlinNoise(float x, float y) {
    // Determine grid cell coordinates
    int x0 = (int)x;
    int x1 = x0 + 1;
    int y0 = (int)y;
    int y1 = y0 + 1;

    // Determine interpolation weights
    // Could also use higher order polynomial/s-curve here
    float sx = x - (float)x0;
    float sy = y - (float)y0;

    // Interpolate between grid point gradients
    float n0, n1, ix0, ix1, value;

    n0 = dotGridGradient(x0, y0, x, y);
    n1 = dotGridGradient(x1, y0, x, y);
    ix0 = interpolate(n0, n1, sx);

    n0 = dotGridGradient(x0, y1, x, y);
    n1 = dotGridGradient(x1, y1, x, y);
    ix1 = interpolate(n0, n1, sx);

    value = interpolate(ix0, ix1, sy);
    return value;
}

/**********************************************************************************
 *
 *                                 Cloud Generators
 *
 ***********************************************************************************/

/**
 * Generates a set of bounding points (corners) for a 3D box
 * and places them into Clouds::positions.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateBoundingPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( positions != nullptr ) { delete positions; }
  positions =  new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;
        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        positions->col( i ) = corner;
      }
    }
  }
}

/**
 * Generates a set of random Worley points for each box in 3D space
 * and places them into Clouds::worley_pts.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateWorleyPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( worley_pts != nullptr ) { delete worley_pts; }
  worley_pts = new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        Vector3f R = Vector3f::Random(3).array().abs();

        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        Vector3f pt = corner + R * cell_size;
        worley_pts->col( i ) = pt;
      }
    }
  }
}

/**
 * Generates a set of density points and assigns them with a value:
 * For each pixel [density point] -> calculate the distance between
 *                                   that pixel and the closest Worley
 *                                   point [density value]
 * Results are placed into Clouds::density_pts & Clouds::density_vals.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateDensityValues(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( density_pts != nullptr ) { delete density_pts; }
  density_pts = new MatrixXf( 3, matrixDimension * matrixDimension );

  if ( density_vals != nullptr ) { delete density_vals; }
  density_vals = new MatrixXf( 3, matrixDimension * matrixDimension );
  float density_cell_size = cell_size / numberOfCells;

  if ( lines != nullptr ) { delete lines; }
  lines = new MatrixXf( 3, 2 * matrixDimension * matrixDimension );

  // for each cell (a cell contains 1 worley point)
  float max_dist = std::numeric_limits<float>::min();
  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        int i = z + y * numberOfCells + x * numberOfCells * numberOfCells; // cell index
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        // for each density block inside the cell
        for ( int xx = 0 ; xx < numberOfCells ; xx++ ) {
          for ( int yy = 0 ; yy < numberOfCells ; yy++ ) {
            for ( int zz = 0 ; zz < numberOfCells ; zz++ ) {
              int j = zz + yy * numberOfCells + xx * numberOfCells * numberOfCells; // density cell index
              Vector3f density_corner = corner + Vector3f( xx, yy, zz ) * density_cell_size;
              Vector3f density_center = density_corner + Vector3f( 1., 1., 1. ) * density_cell_size / 2;

              // search 27 cells for closest Worley point
              const Vector3f start = Vector3f( x - 1, y - 1, z - 1 );
              float min_dist = std::numeric_limits<float>::max();
              Vector3f min_pt;

              for ( int sx = start.x() ; sx < start.x() + 3 ; sx++  ) {
                int sxx = (sx < 0 ? numberOfCells + sx : sx) % numberOfCells;
                for ( int sy = start.y() ; sy < start.y() + 3 ; sy++ ) {
                  int syy = (sy < 0 ? numberOfCells + sy : sy) % numberOfCells;
                  for ( int sz = start.z() ; sz < start.z() + 3 ; sz++ ) {
                    int szz = (sz < 0 ? numberOfCells + sz : sz) % numberOfCells;

                    if ( sx != sxx || sy != syy || sz != szz ) {
                      // wrapping

                      // index into cells w/ wrapping
                      int k = szz + syy * numberOfCells + sxx * numberOfCells * numberOfCells;
                      Vector3f v = worley_pts->col( k );
                      float dist = (density_center - v).norm();

                      for ( int tx = 0, offx = -1 ; tx < numberOfCells ; tx++, offx++ ) {
                        for ( int ty = 0 , offy = -1 ; ty < numberOfCells ; ty++, offy++ ) {
                          for ( int tz = 0 , offz = -1 ; tz < numberOfCells ; tz++, offz++ ) {
                            Vector3f V = v + Vector3f( offx, offy, offz );
                            float dist = (density_center - V).norm();

                            if ( dist < min_dist ) {
                              min_pt = V;
                              min_dist = dist;
                            }
                          }
                        }
                      }

                    } else {
                      // not wrapping

                      int k = sz + sy * numberOfCells + sx * numberOfCells * numberOfCells; // index into cells w/ wrapping
                      Vector3f v = worley_pts->col( k );

                      float dist = (density_center - v).norm();

                      if ( dist < min_dist ) {
                        min_pt = v;
                        min_dist = dist;
                      }
                    }

                  }
                }
              }

              float cell_size = 1. / num_cells;

              // Add perlin noise on xz-plane
              float perlin = perlinNoise( density_center.x() * cell_size, density_center.z() * cell_size );

              int I = i * pow( numberOfCells, 3 ) + j;

              lines->col( 2 * I ) = density_center;
              lines->col( 2 * I + 1 ) = min_pt;

              density_pts->col( I ) = density_center;
              density_vals->col( I ) = Vector3f( perlin, min_dist, 0 );

              // Find maximum value for nomalizing
              if ( min_dist > max_dist ) {
                max_dist = min_dist;
              }
            }
          }
        } // end for each density block inside the cell

      }
    }
  } // end for each cell

  // Normalize Worley Noise
  for ( int i = 0 ; i < density_vals->cols() ; i++ ) {
    density_vals->col(i).y() /= max_dist;
  }
}

/**
 * Main generate functon for Bounding, worley, and density points
 */
void Clouds::generatePoints() {
  generateBoundingPoints(num_cells + 1);
  generateWorleyPoints(num_cells);
  generateDensityValues(num_cells);
}

void Clouds::generateBoundingBox() {
  // 12 lines to draw cube

  Vector3f& a = bbox_min;
  Vector3f& b = bbox_max;

  // back face
  bbox_pts.col( 0  ) = bbox_min;
  bbox_pts.col( 1  ) = Vector3f( b.x() , a.y() , a.z() );

  bbox_pts.col( 2  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_pts.col( 3  ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_pts.col( 4  ) = Vector3f( b.x() , b.y() , a.z() );
  bbox_pts.col( 5  ) = Vector3f( a.x() , b.y() , a.z() );

  bbox_pts.col( 6  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_pts.col( 7  ) = bbox_min;

  // front face
  bbox_pts.col( 8  ) = bbox_max;
  bbox_pts.col( 9  ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_pts.col( 10 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_pts.col( 11 ) = Vector3f( a.x() , a.y() , b.z() );

  bbox_pts.col( 12 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_pts.col( 13 ) = Vector3f( b.x() , a.y() , b.z() );

  bbox_pts.col( 14 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_pts.col( 15 ) = bbox_max;

  // Spokes between squares
  bbox_pts.col( 16 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_pts.col( 17 ) = Vector3f( a.x() , a.y() , b.z() );

  bbox_pts.col( 18 ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_pts.col( 19 ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_pts.col( 20 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_pts.col( 21 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_pts.col( 22 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_pts.col( 23 ) = Vector3f( b.x() , a.y() , a.z() );

  // back face
  bbox_tris.col( 0  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 1  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 2  ) = Vector3f( a.x() , a.y() , a.z() );

  bbox_tris.col( 3  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 4  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 5  ) = Vector3f( b.x() , b.y() , a.z() );

  // front face
  bbox_tris.col( 6  ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 7  ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 8  ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_tris.col( 9  ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 10 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 11 ) = Vector3f( a.x() , b.y() , b.z() );

  // left side
  bbox_tris.col( 12 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_tris.col( 13 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 14 ) = Vector3f( a.x() , b.y() , a.z() );

  bbox_tris.col( 15 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 16 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 17 ) = Vector3f( a.x() , b.y() , a.z() );

  // right side
  bbox_tris.col( 18 ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 19 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 20 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_tris.col( 21 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 22 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 23 ) = Vector3f( b.x() , b.y() , a.z() );

  // top
  bbox_tris.col( 24 ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 25 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 26 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_tris.col( 27 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 28 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 29 ) = Vector3f( b.x() , b.y() , a.z() );

  // top
  bbox_tris.col( 30 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_tris.col( 31 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 32 ) = Vector3f( b.x() , a.y() , a.z() );

  bbox_tris.col( 33 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 34 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 35 ) = Vector3f( b.x() , a.y() , a.z() );
}

/**
 * Fill the texture with 1 - d
 *   where d is the distance to the nearest Worley point
 */
void Clouds::generateDensityTexture() {
  glActiveTexture( GL_TEXTURE0 + density_tex_unit );
  glBindTexture( GL_TEXTURE_3D, density_tex_id );

  // set the texture wrapping/filtering options (on the currently bound texture object)
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // load and generate the texture
  glTexImage3D(
      GL_TEXTURE_3D,
      0,         // mipmap level ?
      GL_RGB,    // internal format
      num_cells, // width
      num_cells, // height
      num_cells, // depth
      0,         // border
      GL_RGB,    // format
      GL_UNSIGNED_BYTE,  // type
      density_vals->data() );
}
