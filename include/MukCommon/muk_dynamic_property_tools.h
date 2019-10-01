#pragma once

// function setter / getter
#define MUK_SET(type, set) [&] (type d) { set(d); }
#define MUK_GET(get) [&] () { return get(); }
#define MUK_C_SET(type, set) [&] (const type& d) { set(d); }
#define MUK_C_GET(type, get) [&] () -> const type& { return get(); }
// direct set and return
#define MUK_D_SET(type, var) [&] (type d) { var = d; }
#define MUK_D_GET(var)       [&] { return var; }
#define MUK_D_C_SET(type, var) [&] (const type& d) { var = d; }
#define MUK_D_C_GET(type, var) [&] () -> const type&  { return var; }
