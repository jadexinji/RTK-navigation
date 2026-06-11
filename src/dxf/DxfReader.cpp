#include "dxf/DxfReader.h"

#include <stdexcept>

namespace rtk_nav {

bool DxfReader::backendAvailable() {
    return false;
}

std::string DxfReader::backendStatus() {
    return "DXF geometry model is ready, but no parser backend is linked. "
           "The planned backend is libdxfrw.";
}

std::vector<MapFeature> DxfReader::load(const std::string& dxf_path) const {
    throw std::runtime_error(
        "Cannot read DXF file '" + dxf_path + "': " + backendStatus());
}

}  // namespace rtk_nav
