#include "modules/selection/catalog/Stage1ComponentCatalog.h"

namespace RoboSDP::Selection::Catalog
{

const std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto>& Stage1ComponentCatalog::Motors() const
{
    return m_motors;
}

const std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto>& Stage1ComponentCatalog::Reducers() const
{
    return m_reducers;
}

} // namespace RoboSDP::Selection::Catalog
