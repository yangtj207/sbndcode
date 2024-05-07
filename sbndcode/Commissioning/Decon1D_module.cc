////////////////////////////////////////////////////////////////////////
// Class:       Decon1D
// Plugin Type: producer (Unknown Unknown)
// File:        Decon1D_module.cc
//
// Generated at Mon May  6 11:21:55 2024 by Tingjun Yang using cetskelgen
// from cetlib version 3.18.02.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDProducer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include "lardataobj/RecoBase/Wire.h"
#include "lardataobj/RawData/RawDigit.h"
#include "lardataobj/RawData/raw.h"
#include "larcore/Geometry/Geometry.h"

#include "TFile.h"
#include "TH1D.h"
#include "TComplex.h"
#include "TVirtualFFT.h"

#include <memory>
#include <vector>
#include <complex>
#include <iostream>
#include <fftw3.h>

namespace sbnd {
  class Decon1D;
}

using namespace std;

class sbnd::Decon1D : public art::EDProducer {
public:
  explicit Decon1D(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  Decon1D(Decon1D const&) = delete;
  Decon1D(Decon1D&&) = delete;
  Decon1D& operator=(Decon1D const&) = delete;
  Decon1D& operator=(Decon1D&&) = delete;

  // Required functions.
  void produce(art::Event& e) override;

private:

  // Declare member data here.
  art::InputTag fRawDigitModuleLabel;
  std::string   fResponseFile;
  vector<vector<TComplex>> vresp;

};


sbnd::Decon1D::Decon1D(fhicl::ParameterSet const& p)
  : EDProducer{p}  // ,
  , fRawDigitModuleLabel{p.get<art::InputTag>("RawDigitModuleLabel")}
  , fResponseFile(p.get<std::string>("ResponseFile"))
{
  produces<std::vector<recob::Wire>>();

  vresp.resize(6);

  // Get response functions
  std::string fullname;
  cet::search_path sp("FW_SEARCH_PATH");
  sp.find_file(fResponseFile, fullname);
  TFile *resfile;
  if (fullname.empty()) {
    throw cet::exception("sbnd::Decon1D") << "Unable to find response file "  << fResponseFile;
  }
  else{
    resfile = TFile::Open(fullname.c_str());
    TH1D *hre[6];
    TH1D *him[6];
    for (int i = 0; i<6; ++i){
      hre[i] = (TH1D*)resfile->Get(Form("re%d",i));
      him[i] = (TH1D*)resfile->Get(Form("im%d",i));
      if (!hre[i]) throw cet::exception("sbnd::Decon1D") << "Unable to find response "<<Form("re%d",i);
      if (!him[i]) throw cet::exception("sbnd::Decon1D") << "Unable to find response "<<Form("im%d",i);
      for (int j = 1; j<=3415; ++j){
        vresp[i].push_back(TComplex(hre[i]->GetBinContent(j),
                                    him[i]->GetBinContent(j)));
      }
    }
  }

}

void sbnd::Decon1D::produce(art::Event& e)
{
  auto out_recowaveforms = std::make_unique< std::vector< recob::Wire > >();

  art::ServiceHandle<geo::Geometry> geo;

  int n = 3415;

  TVirtualFFT *fftr2c = TVirtualFFT::FFT(1,&n,"R2C ES K");
  TVirtualFFT *fftc2r = TVirtualFFT::FFT(1,&n,"C2R ES K");

  // Get raw digits
  auto const& rawdigts = e.getProduct<std::vector<raw::RawDigit>>(fRawDigitModuleLabel);
  for (const auto & rd : rawdigts){
    std::vector<short> rawadc;      //UNCOMPRESSED ADC VALUES.
    rawadc.resize(rd.Samples());
    raw::Uncompress(rd.ADCs(), rawadc, rd.GetPedestal(), rd.Compression());
    int ch = rd.Channel();
    auto const & chids = geo->ChannelToWire(ch);
    int plane = chids[0].Plane;
    int tpc = chids[0].TPC;
    for (size_t i = 0; i<rawadc.size(); ++i){
      fftr2c->SetPoint(i, rawadc[i] - rd.GetPedestal());
    }
    fftr2c->Transform();
    for (size_t i = 0; i<rawadc.size()/2+1; ++i){
      double re, im;
      fftr2c->GetPointComplex(i, re, im);
      TComplex point = TComplex(re, im)*vresp[plane+3*tpc][i];
      fftc2r->SetPointComplex(i,point);
    }
    fftc2r->Transform();
    std::vector<float> sigs;
    for (int i = 0; i<n; ++i){
      sigs.push_back(fftc2r->GetPointReal(i)/n*100);
    }
    recob::Wire::RegionsOfInterest_t rois(n);
    rois.add_range(0, std::move(sigs));
    out_recowaveforms->emplace_back(recob::Wire(rois, rd.Channel(), geo->View(rd.Channel())));
  }
  e.put(std::move(out_recowaveforms));
}

DEFINE_ART_MODULE(sbnd::Decon1D)
