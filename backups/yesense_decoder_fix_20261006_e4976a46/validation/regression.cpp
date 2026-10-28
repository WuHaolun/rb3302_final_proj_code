#include <cassert>
#include <cstdio>
#include <cstdint>
#include <vector>
#include "yesense_decoder.h"
#include "yesense_decoder_comm.h"
using Bytes=std::vector<unsigned char>;
using yesense::yis_out_data_t;
Bytes packet(const Bytes &payload, bool bad=false){
  Bytes p={0x59,0x53,1,0,(unsigned char)payload.size()};
  p.insert(p.end(),payload.begin(),payload.end());
  unsigned char a=0,b=0;
  for(size_t i=2;i<p.size();++i){a+=p[i];b+=a;}
  p.push_back(a^(bad?1:0));p.push_back(b);return p;
}
Bytes imu(){
  Bytes p;
  for(auto id:{0x10,0x20}){p.push_back(id);p.push_back(12);p.insert(p.end(),12,0);}
  p.push_back(0x41);p.push_back(16);
  for(int32_t v:{1000000,0,0,0})for(int i=0;i<4;i++)p.push_back((uint32_t(v)>>(8*i))&255);
  return p;
}
int feed(yesense::yesense_decoder &d,Bytes p,yis_out_data_t &o){return d.data_proc(p.data(),p.size(),&o);}
void valid(const yis_out_data_t &o){assert(o.content.acc&&o.content.gyro&&o.content.quat);assert(o.quat.q0>0.999f);}
int main(){
  const auto payload=imu();const auto good=packet(payload);int checks=0;
  {yesense::yesense_decoder d;yis_out_data_t o{};assert(feed(d,good,o)==analysis_ok);valid(o);checks++;}
  for(size_t split=1;split<good.size();split++){
    yesense::yesense_decoder d;yis_out_data_t o{};
    assert(feed(d,Bytes(good.begin(),good.begin()+split),o)!=analysis_ok);
    assert(feed(d,Bytes(good.begin()+split,good.end()),o)==analysis_ok);valid(o);checks++;
  }
  {yesense::yesense_decoder d;yis_out_data_t o{};
    feed(d,Bytes{0,0,0x59},o);
    assert(feed(d,Bytes(good.begin()+1,good.end()),o)==analysis_ok);valid(o);checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};Bytes bad=packet(payload,true);bad.insert(bad.begin(),{0,0});
    feed(d,bad,o);for(int i=0;i<50;i++){assert(feed(d,good,o)==analysis_ok);valid(o);}checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};
    assert(feed(d,good,o)==analysis_ok);feed(d,packet(Bytes{1,2,0,0},true),o);
    assert(feed(d,good,o)==analysis_ok);valid(o);checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};Bytes b;
    for(int i=0;i<8;i++)b.insert(b.end(),good.begin(),good.end());
    assert(feed(d,b,o)==analysis_ok);valid(o);
    for(int i=1;i<8;i++){assert(d.data_proc(nullptr,0,&o)==analysis_ok);valid(o);}
    assert(d.data_proc(nullptr,0,&o)!=analysis_ok);checks++;}
  for(const auto &broken:std::vector<Bytes>{{0x10,12,0},{0x20}}){
    yesense::yesense_decoder d;yis_out_data_t o{};feed(d,packet(broken),o);
    assert(feed(d,good,o)==analysis_ok);valid(o);checks++;
  }
  {yesense::yesense_decoder d;yis_out_data_t o{};Bytes p={0x99,4,0x10,12,0x59,0x53};p.insert(p.end(),payload.begin(),payload.end());
    assert(feed(d,packet(p),o)==analysis_ok);valid(o);checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};
    assert(feed(d,Bytes(1025,0),o)==buf_full);
    assert(feed(d,good,o)==analysis_ok);valid(o);checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};
    assert(d.data_proc(nullptr,1,&o)==para_err);
    assert(d.data_proc(nullptr,0,nullptr)==para_err);
    assert(feed(d,good,o)==analysis_ok);checks++;}
  {yesense::yesense_decoder d;yis_out_data_t o{};uint32_t rng=12345;
    for(int i=0;i<1000;i++){
      Bytes noise;
      for(int j=0;j<(i%23)+1;j++){rng=rng*1664525u+1013904223u;noise.push_back((rng>>24)&255);}
      auto bad=packet(payload,true);noise.insert(noise.end(),bad.begin(),bad.end());feed(d,noise,o);
      // Padding completes any false header in random noise, permitting resync.
      feed(d,Bytes(300,0),o);
      assert(feed(d,good,o)==analysis_ok);valid(o);
    }checks++;
  }
  printf("PASS: %d regression cases, including 1000 noise/CRC recovery cycles\n",checks);
}
