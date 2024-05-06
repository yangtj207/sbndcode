////////////////////////////////////////////////////////////////////////
// Class:       TPCWFAna
// Plugin Type: analyzer
// File:        TPCWFAna_module.cc
// Author:      tjyang@fnal.gov
//
// Analyze TPC waveforms
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include "art_root_io/TFileService.h"

#include "lardataobj/RawData/RawDigit.h"
#include "lardataobj/RawData/raw.h"
#include "larcore/Geometry/Geometry.h"

#include "TH1D.h"

namespace sbnd {
  class TPCWFAna;
}


class sbnd::TPCWFAna : public art::EDAnalyzer {
public:
  explicit TPCWFAna(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  TPCWFAna(TPCWFAna const&) = delete;
  TPCWFAna(TPCWFAna&&) = delete;
  TPCWFAna& operator=(TPCWFAna const&) = delete;
  TPCWFAna& operator=(TPCWFAna&&) = delete;

  // Required functions.
  void analyze(art::Event const& e) override;

  // Selected optional functions.
  void beginJob() override;

private:

  // Declare member data here.
  art::InputTag fRawDigitModuleLabel;
  TH1D *hwf[3];
  TH1D *hnoise[3][230];

};


sbnd::TPCWFAna::TPCWFAna(fhicl::ParameterSet const& p)
  : EDAnalyzer{p}  // ,
  , fRawDigitModuleLabel{p.get<art::InputTag>("RawDigitModuleLabel")}
  // More initializers here.
{
  // Call appropriate consumes<>() for any products to be retrieved by this module.
}

void sbnd::TPCWFAna::analyze(art::Event const& e)
{
  // Implementation of required member function here.

  art::ServiceHandle<geo::Geometry> geo;

  int nwfs[3] = {0};

  // Get raw digits
  auto const& rawdigts = e.getProduct<std::vector<raw::RawDigit>>(fRawDigitModuleLabel);
  for (const auto & rd : rawdigts){
    std::vector<short> rawadc;      //UNCOMPRESSED ADC VALUES.
    rawadc.resize(rd.Samples());
    raw::Uncompress(rd.ADCs(), rawadc, rd.GetPedestal(), rd.Compression());
    int ch = rd.Channel();
    auto const & chids = geo->ChannelToWire(ch);
    int tpc = chids[0].TPC;
    int plane = chids[0].Plane;
    int wire = chids[0].Wire;
    int idx = -1;
    if (tpc== 0 && plane == 0 && 
        (wire >= 235 && wire <= 432)
//        ( (wire >= 235 && wire <= 432) ||
//          (wire >= 461 && wire <= 541) ||
//          (wire >= 552 && wire <= 599) ||
//          (wire >= 618 && wire <= 671) ||
//          (wire >= 953 && wire <= 1196)||
//          (wire >= 1299 && wire <= 1455))
        ){
      idx = 0;
    }
    if (tpc == 0 && plane == 1 &&
        (wire >=580 && wire <= 900)){
      //        (wire >=580 && wire <= 926)){
      idx = 1;
    }
    if (tpc == 0 && plane == 2 &&
        (wire >=505 && wire <= 665)){
      //        (wire >=580 && wire <= 926)){
      idx = 2;
    }
    if (idx==0 || idx==1){
      double minadc = 1e10;
      int minbin = -1;
      for (size_t i = 0; i<rawadc.size(); ++i){
        if (rawadc[i]<minadc){
          minbin = i;
          minadc = rawadc[i];
        }
      }
      if (minbin!=-1){
        for (int i = 1; i<=700; ++i){
          double binc = hwf[idx]->GetBinContent(i);
          hwf[idx]->SetBinContent(i, binc + rawadc[minbin-200+i] - rd.GetPedestal());
        }
      }
    }
    if (idx==2){
      double maxadc = -1e10;
      int maxbin = -1;
      for (size_t i = 0; i<rawadc.size(); ++i){
        if (rawadc[i]>maxadc){
          maxbin = i;
          maxadc = rawadc[i];
        }
      }
      if (maxbin!=-1){
        for (int i = 1; i<=700; ++i){
          double binc = hwf[idx]->GetBinContent(i);
          hwf[idx]->SetBinContent(i, binc + rawadc[maxbin-200+i] - rd.GetPedestal());
        }
      }
    }
    if (idx!=-1){
      ++nwfs[idx];
    }
    int idx1 = -1;
    int idx2 = -1;
    if (tpc == 0 && plane == 0){
      idx1 = 0;
      if (wire >= 1650 && wire <1880){
        idx2 = wire - 1650;
      }
    }
    if (tpc == 0 && plane == 1){
      idx1 = 1;
      if (wire >= 20 && wire <250){
        idx2 = wire - 20;
      }
    }
    if (tpc == 0 && plane == 2){
      idx1 = 2;
      if (wire >= 1430 && wire < 1660){
        idx2 = wire - 1430;
      }
    }
    if (idx1 != -1 && idx2 != -1){
      for (size_t i = 0; i<rawadc.size(); ++i){
        hnoise[idx1][idx2]->SetBinContent(i+1, rawadc[i] - rd.GetPedestal());
      }
    }
  }
  for (int i = 0; i<3; ++i){
    hwf[i]->Scale(1./nwfs[i]);
  }
}

void sbnd::TPCWFAna::beginJob()
{
  // Implementation of optional member function here.
  art::ServiceHandle<art::TFileService> tfs;
  for (int i = 0; i<3; ++i){
    hwf[i] = tfs->make<TH1D>(Form("hwf%d",i), Form("hwf%d",i), 700,0,700);
    for (int j = 0; j<230; ++j){
      hnoise[i][j] = tfs->make<TH1D>(Form("hnoise%d_%d",i,j),Form("hnoise%d_%d",i,j), 3415, 0, 3415);
    }
  }

}

DEFINE_ART_MODULE(sbnd::TPCWFAna)
