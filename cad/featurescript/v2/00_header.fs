FeatureScript 3095;
import(path : "onshape/std/common.fs", version : "3095.0");

/* =========================================================================
 * APEX 2026 (REBUILT) robot CAD library, v2.
 *
 * Robot frame: origin at frame centre on the carpet, +X = intake (front),
 * +Y = left, +Z = up, inches. Frame 27.0 x 28.0 in (110 in perimeter).
 *
 * Purchased parts are real vendor CAD instanced from public Onshape
 * documents at pinned versions (SDS MK5n from SDS's Parasolid release,
 * Kraken X60/X44 and electronics from the MKCad library). Machined parts are
 * modelled to drawing level: filleted profiles, #10 clearance holes, bearing
 * bores, hex bores, truss pockets, tube hole patterns, hardware.
 * ========================================================================= */

// ---- purchased parts (public documents, pinned versions)
MK5NA::import(path : "474f5abaa97b996c4239c952/6801bb9c2bdb14593c894316/fe499d3a98b420584aaa9828", version : "09814a5b60ca62d3ad3ce271");
MK5NB::import(path : "474f5abaa97b996c4239c952/6801bb9c2bdb14593c894316/0b9d638745a6d7758a6f9fe4", version : "93ef2ad29be10c1d92cb6884");
KX60::import(path : "2979144bb1e87ddd1747e172/c5c9969e813e179c431de156/1ce7f6edfe09be4fb53cad38", version : "4bf12c52a8e7e3e06f55f01a");
KX44::import(path : "2979144bb1e87ddd1747e172/c5c9969e813e179c431de156/52e90b54bef81dd7399a2cc8", version : "7be6988839a8d46196327982");
// electronics (MKCad Electronics Parts V114 / Electronics V80 / Sensors V36)
LL4::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/3a5e019812d791d4cf659731", version : "d6135c63c7d3db7a0902b467");
CANR::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/664d987a2bb629af41a4e3bf", version : "ab511519a40ed098fe45e3d7");
RIO2::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/69380cdc05d2981bfa6ba2ff", version : "090022aa17f06a59d8659975");
PDH::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/611b36b0cef04e18a3fe6800", version : "e3aec2618671be24e7e3cac1");
MPM::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/85eb446dcfbcfa9bd9ebe269", version : "b047db0e7baa0a347069e8ec");
RSL::import(path : "40c7074f3a9150f7c3476a13/f3bf32edd8ceb438992a2696/22c2ed2460b61913bd336df9", version : "9292cf00dad6f842b61a60e8");
CANIV::import(path : "11dea59fcde97bd96bad66d1/182f9d857e4be3d2bf66fc97/0a4396c222b6510c47e7db15", version : "7e6a31b54586b199e1c2263f");
BATT::import(path : "11dea59fcde97bd96bad66d1/182f9d857e4be3d2bf66fc97/80abd36504fe0fcb96faf90a", version : "f85da378c0ba25dc78bf097b");
BRKR::import(path : "11dea59fcde97bd96bad66d1/182f9d857e4be3d2bf66fc97/66286c7754dccde30cf38fe3", version : "710973c25190dc5598634f2e");
VH109::import(path : "11dea59fcde97bd96bad66d1/182f9d857e4be3d2bf66fc97/845c489d53e65ffb40218706", version : "15d05b859a37884817c67e56");
SWCH::import(path : "11dea59fcde97bd96bad66d1/182f9d857e4be3d2bf66fc97/fe77be7bfb890dcdc66cfd09", version : "5478f1082452256a0eb85cff");
PIG2::import(path : "e83eca3cba69ccda678d5a93/a880faa858fb1254141268ec/9726460f15457e43200a7079", version : "02e2652ed0321afdeabfa263");
