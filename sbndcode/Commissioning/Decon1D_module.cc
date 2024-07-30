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

#include "sbndcode/Utilities/DigitalNoiseChannelStatus.h"
#include "sbndcode/ChannelMaps/TPC/TPCChannelMapService.h"

#include "TFile.h"
#include "TH1D.h"
#include "TComplex.h"
#include "TVirtualFFT.h"

#include <memory>
#include <vector>
#include <complex>
#include <iostream>

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
  vector<float> normalization;
  vector<float> roi_threshold;
  vector<float> sigprotection;
  vector<unsigned int>   excluchannels;
};


sbnd::Decon1D::Decon1D(fhicl::ParameterSet const& p)
  : EDProducer{p}  // ,
  , fRawDigitModuleLabel{p.get<art::InputTag>("RawDigitModuleLabel")}
  , fResponseFile(p.get<std::string>("ResponseFile"))
  , normalization(p.get<vector<float>>("normalization"))
  , roi_threshold(p.get<vector<float>>("roi_threshold"))
  , sigprotection(p.get<vector<float>>("sigprotection"))
  , excluchannels(p.get<vector<unsigned int>>("excluchannels"))
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
  art::ServiceHandle<sbnd::DigitalNoiseChannelStatus> chstatus;
  art::ServiceHandle<SBND::TPCChannelMapService> channelMap;

  int n = 3415;

  TVirtualFFT *fftr2c = TVirtualFFT::FFT(1,&n,"R2C ES K");
  TVirtualFFT *fftc2r = TVirtualFFT::FFT(1,&n,"C2R ES K");

  // Get raw digits
  auto const& rawdigits = e.getProduct<std::vector<raw::RawDigit>>(fRawDigitModuleLabel);

  // make a big map of mean ADC values for each plane and FEMB for each tick.
  // for correlated noise removing
  // first index -- femb, second index, plane, third index: tick

  unordered_map<int, unordered_map<int, unordered_map<int, double>>> meanmap;

  for (int itick = 0; itick < n; ++itick){

    unordered_map<int, unordered_map<int, vector<double>>> adcvmap;

    for (size_t ichan=0; ichan<rawdigits.size(); ++ichan){

      unsigned int ic = rawdigits[ichan].Channel();
      auto const & ci = channelMap->GetChanInfoFromOfflChan(ic);
      if (chstatus->IsBad(ic)) continue;
      bool exclu = false;
      for (auto const & ch : excluchannels){
        if (ic == ch) exclu = true;
      }
      if (exclu) continue;
      auto const & chids = geo->ChannelToWire(ic);
      int fembident = 100*ci.WIBCrate + 10*ci.WIB + ci.FEMBOnWIB;
      // to do -- remove ROIs with signals
      if (abs(rawdigits[ichan].ADC(itick) - rawdigits[ichan].GetPedestal())<sigprotection[chids[0].TPC*3+chids[0].Plane]){
        adcvmap[fembident][ci.plane].push_back(rawdigits[ichan].ADC(itick) - rawdigits[ichan].GetPedestal());
      }
    }
	    
    for (const auto& fp : adcvmap){  // loop over FEMBs
      
      for (const auto& pp : fp.second){  // loop over planes

        double adcmed = TMath::Mean(pp.second.size(),pp.second.data());
        meanmap[fp.first][pp.first][itick] = adcmed;
      }
    }
  }


  for (const auto & rd : rawdigits){
    std::vector<short> rawadc;      //UNCOMPRESSED ADC VALUES.
    rawadc.resize(rd.Samples());
    raw::Uncompress(rd.ADCs(), rawadc, rd.GetPedestal(), rd.Compression());
    std::vector<float> adc_fft;
    std::vector<bool> inroi(n,false);
    recob::Wire::RegionsOfInterest_t rois(n);
    unsigned int ic = rd.Channel();
    bool exclu = false;
    for (auto const & ch : excluchannels){
      if (ic == ch) exclu = true;
    }
    if (exclu) continue;
    auto const & ci = channelMap->GetChanInfoFromOfflChan(ic);
    int fembident = 100*ci.WIBCrate + 10*ci.WIB + ci.FEMBOnWIB;
    auto const & chids = geo->ChannelToWire(ic);
    int plane = chids[0].Plane;
    int tpc = chids[0].TPC;

    if (chstatus->IsBad(ic)){
      //cout<<"Bad channel. TPC = "<<tpc<<" "<<plane<<" "<<chids[0].Wire<<endl;
//      for (int i = 0; i<n; ++i){
//        adc_fft.push_back(0);
//      }
//      rois.add_range(0, std::move(adc_fft));
//      out_recowaveforms->emplace_back(recob::Wire(rois, rd.Channel(), geo->View(rd.Channel())));
      continue;
    }
    for (size_t i = 0; i<rawadc.size(); ++i){
      fftr2c->SetPoint(i, rawadc[i] - rd.GetPedestal() - meanmap[fembident][plane][i]);
      //fftr2c->SetPoint(i, rawadc[i] - rd.GetPedestal());
    }
    fftr2c->Transform();
    for (size_t i = 0; i<rawadc.size()/2+1; ++i){
      double re, im;
      fftr2c->GetPointComplex(i, re, im);
      TComplex point = TComplex(re, im)*vresp[plane+3*tpc][i];
      fftc2r->SetPointComplex(i,point);
    }
    fftc2r->Transform();
    for (int i = 0; i<n; ++i){
      adc_fft.push_back(fftc2r->GetPointReal(i)/n*normalization[plane+3*tpc]);
    }
    for (int i = 0; i<n; ++i){
      if (adc_fft[i] > roi_threshold[plane+3*tpc]){
        for (int j = i - 100; j < i + 100; ++j){
          if (j>=0 && j<n){
            inroi[j] = true;
          }
        }
      }
    }

    std::vector<float> sigs;
    int lastsignaltick = -1;
    int roistart = -1;
    bool hasROI = false;
    for (int i = 0; i < n; ++i) {
      if (inroi[i]) {
        hasROI = true;
        if (sigs.empty()) {
          sigs.push_back(adc_fft[i]);
          lastsignaltick = i;
          roistart = i;
        }
        else {
          if (int(i) != lastsignaltick + 1) {
            rois.add_range(roistart, std::move(sigs));
            sigs.clear();
            sigs.push_back(adc_fft[i]);
            lastsignaltick = i;
            roistart = i;
          }
          else {
            sigs.push_back(adc_fft[i]);
            lastsignaltick = i;
          }
        }
      }
    }    
    if (!sigs.empty()) { rois.add_range(roistart, std::move(sigs)); }
    if (hasROI) out_recowaveforms->emplace_back(recob::Wire(rois, rd.Channel(), geo->View(rd.Channel())));
  }
  e.put(std::move(out_recowaveforms));
}

DEFINE_ART_MODULE(sbnd::Decon1D)
