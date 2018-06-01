#ifndef SPIRE_ASYNC_FILESYSTEMDUMMY_HPP
#define SPIRE_ASYNC_FILESYSTEMDUMMY_HPP

#include <es-log/trace-log.h>
#include <vector>
#include "Filesystem.hpp"
#include <spire/scishare.h>

namespace spire {

// Use this system if you want to provide synchronous operations on disk.
// Only used when disk is present.
class FilesystemDummy : public Filesystem
{
public:
  FilesystemDummy(const std::string& prefix = "")  {mPrefix = prefix;}
  virtual ~FilesystemDummy()                       {}

  void update() override;

  void readFile(const std::string& assetName, const FSCallback& cb) override;

private:

  // Vector of assets to load when the system update arrives.
  struct AssetItem
  {
    AssetItem(const std::string& asset, const FSCallback& cb) :
        assetName(asset),
        callback(cb)
    {}

    std::string assetName;
    FSCallback  callback;
  };

  // Assets that are to be loaded.
  std::vector<AssetItem>  mAssets;
  std::string             mPrefix;
};

} // namespace spire

#endif
