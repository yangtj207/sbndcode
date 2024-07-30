////////////////////////////////////////////////////////////////////////
// Class:       TrackDumper
// Plugin Type: analyzer (Unknown Unknown)
// File:        TrackDumper_module.cc
//
// Generated at Mon Jul  8 18:48:59 2024 by Tingjun Yang using cetskelgen
// from cetlib version 3.18.02.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "lardataobj/RecoBase/Track.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace sbnd {
  class TrackDumper;
}


class sbnd::TrackDumper : public art::EDAnalyzer {
public:
  explicit TrackDumper(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  TrackDumper(TrackDumper const&) = delete;
  TrackDumper(TrackDumper&&) = delete;
  TrackDumper& operator=(TrackDumper const&) = delete;
  TrackDumper& operator=(TrackDumper&&) = delete;

  // Required functions.
  void analyze(art::Event const& e) override;

private:

  // Declare member data here.
  art::InputTag fTrackModuleLabel;
  // Declare member data here.

};


sbnd::TrackDumper::TrackDumper(fhicl::ParameterSet const& p)
  : EDAnalyzer{p}
  , fTrackModuleLabel{p.get<art::InputTag>("TrackModuleLabel")}
  // More initializers here.
{
  // Call appropriate consumes<>() for any products to be retrieved by this module.
}

void sbnd::TrackDumper::analyze(art::Event const& e)
{
  // Implementation of required member function here.
  auto const& tracks = e.getProduct<std::vector<recob::Track>>(fTrackModuleLabel);
  ofstream outfile ("trk.txt");  
  for (const auto & trk : tracks){
    for (size_t i = 0; i<trk.NPoints(); ++i){
      if (trk.HasValidPoint(i)){
        outfile<<trk.TrajectoryPoint(i).position.X()<<" "
               <<trk.TrajectoryPoint(i).position.Y()<<" "
               <<trk.TrajectoryPoint(i).position.Z()<<" "
               <<"1 "<<trk.ID()<<endl;
      }
    }
  }
  outfile.close();
}

DEFINE_ART_MODULE(sbnd::TrackDumper)
