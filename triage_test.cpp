// Triage rule test.
//
// Do not run directly - run test/triage_extract_and_test.py, which slices
// triage_eval() out of the shipped .ino first, so this can never drift from
// the rules the firmware actually uses.
//
// Covers the balanced CRITICAL policy, that microphones are not down-ranked
// relative to cameras, that declared SPEAKERS and phones are not treated as
// recorders, the HIGH tier, the hard cap that keeps anything outside the gate
// at MEDIUM however good its signature, and that all 192 rule combinations
// return a non-empty reason string.
#include "TRIAGE_UNDER_TEST_H"

static int fails=0;
static void chk(bool c,const char*w){printf("  [%s] %s\n",c?"PASS":"FAIL",w);if(!c)fails++;}
static risk_t ev(tier_t tb,dtype_t d,uint8_t tier,uint32_t evd,const char**why){
  track_t t; memset(&t,0,sizeof(t));
  t.tier_best=tb; t.dtype=d; t.dtype_tier=tier; t.evidence=evd;
  return triage_eval(&t,why);
}
int main(){
  const char*w;
  printf("\n== CRITICAL: balanced policy ==\n");
  chk(ev(TIER_NEAR,DTYPE_CAMERA,DT_TIER_A,0,&w)==RISK_CRITICAL,
      "WPS-declared camera at NEAR -> CRITICAL (no baseline needed)");
  chk(ev(TIER_NEAR,DTYPE_MIC,DT_TIER_A,0,&w)==RISK_CRITICAL,
      "BLE-declared mic at NEAR -> CRITICAL");
  chk(ev(TIER_NEAR,DTYPE_CAMERA,DT_TIER_B,0,&w)==RISK_CRITICAL,
      "streaming behaviour camera at NEAR -> CRITICAL");
  chk(ev(TIER_CONTACT,DTYPE_CAMERA,DT_TIER_D,0,&w)==RISK_CRITICAL,
      "vendor-hint camera at CONTACT -> CRITICAL");
  chk(ev(TIER_NEAR,DTYPE_CAMERA,DT_TIER_C,E_ROOM_ONLY,&w)==RISK_CRITICAL,
      "name-matched camera, NEAR + room-only -> CRITICAL");

  printf("\n== mics are not down-ranked vs cameras ==\n");
  chk(ev(TIER_CONTACT,DTYPE_MIC,DT_TIER_A,0,&w)==
      ev(TIER_CONTACT,DTYPE_CAMERA,DT_TIER_A,0,&w),
      "mic and camera score identically at contact");

  printf("\n== speakers must NOT be treated as recorders ==\n");
  chk(ev(TIER_NEAR,DTYPE_SPEAKER,DT_TIER_A,0,&w)<RISK_CRITICAL,
      "declared Audio Sink (speaker) at NEAR is not CRITICAL");
  chk(ev(TIER_NEAR,DTYPE_PHONE,DT_TIER_A,0,&w)<RISK_CRITICAL,
      "declared phone at NEAR is not CRITICAL");

  printf("\n== HIGH ==\n");
  chk(ev(TIER_NEAR,DTYPE_CAMERA,DT_TIER_D,0,&w)==RISK_HIGH,
      "vendor-hint camera at NEAR -> HIGH");
  chk(ev(TIER_CONTACT,DTYPE_UNKNOWN,DT_TIER_NONE,0,&w)==RISK_HIGH,
      "unidentified device at CONTACT -> HIGH");
  chk(ev(TIER_NEAR,DTYPE_UNKNOWN,DT_TIER_NONE,E_ROOM_ONLY,&w)==RISK_HIGH,
      "unidentified, NEAR + room-only -> HIGH");

  printf("\n== ambient is capped, whatever the signature says ==\n");
  chk(ev(TIER_AMBIENT,DTYPE_CAMERA,DT_TIER_A,0,&w)==RISK_MEDIUM,
      "declared camera OUTSIDE the gate caps at MEDIUM");
  chk(ev(TIER_AMBIENT,DTYPE_CAMERA,DT_TIER_A,E_ROOM_ONLY|E_STREAM_PROFILE,&w)
      ==RISK_MEDIUM, "even with every other flag, ambient caps at MEDIUM");
  chk(ev(TIER_AMBIENT,DTYPE_UNKNOWN,DT_TIER_NONE,0,&w)==RISK_LOW,
      "nothing at all -> LOW");

  printf("\n== every level carries a reason string ==\n");
  bool allwhy=true;
  tier_t tiers[]={TIER_AMBIENT,TIER_NEAR,TIER_CONTACT};
  dtype_t ds[]={DTYPE_UNKNOWN,DTYPE_CAMERA,DTYPE_MIC,DTYPE_SPEAKER};
  for(auto tb:tiers) for(auto d:ds) for(uint8_t ti=0;ti<4;ti++)
    for(uint32_t e=0;e<4;e++){
      const char*r=NULL;
      ev(tb,d,ti,(e&1?E_ROOM_ONLY:0)|(e&2?E_STREAM_PROFILE:0),&r);
      if(!r||!r[0]) allwhy=false;
    }
  chk(allwhy,"all 192 rule combinations return a non-empty reason");

  printf("\n%s (%d failures)\n\n",fails?"TESTS FAILED":"ALL TRIAGE TESTS PASSED",fails);
  return fails?1:0;
}
