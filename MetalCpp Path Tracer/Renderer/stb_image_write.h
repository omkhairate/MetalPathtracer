#ifndef STB_IMAGE_WRITE_H
#define STB_IMAGE_WRITE_H

#include <fstream>

inline int stbi_write_png(const char *filename, int w, int h, int comp,
                          const void *data, int stride_bytes) {
    if (comp < 3) return 0;
    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) return 0;
    f << "P6\n" << w << " " << h << "\n255\n";
    const unsigned char *row = static_cast<const unsigned char *>(data);
    for (int y = 0; y < h; ++y) {
        f.write(reinterpret_cast<const char*>(row), w * 3);
        row += stride_bytes;
    }
    return 1;
}

#endif // STB_IMAGE_WRITE_H
