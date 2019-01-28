#ifndef _FSM_H_
#define _FSM_H_

#include <vector>
#include <string>

using namespace std;

namespace {
namespace fsm {

    string STATE_KL = "KL"; // keep lane
    string STATE_CLL = "CLL"; // change lane left
    string STATE_CLR = "CLR"; // change lane right

    bool is_KL(const string& state) {
        return state.compare(STATE_KL) == 0;
    }

    bool is_CLL(const string& state) {
        return state.compare(STATE_CLL) == 0;
    }

    bool is_CLR(const string& state) {
        return state.compare(STATE_CLR) == 0;
    }

    vector<string> successor_states(const string& state) {
        vector<string> sstates;

        if (is_KL(state)) {
            sstates.push_back(STATE_KL);
            sstates.push_back(STATE_CLL);
            sstates.push_back(STATE_CLR);
        }
        else if (is_CLL(state)) {
           sstates.push_back(STATE_KL); 
           sstates.push_back(STATE_CLL);
        }
        else if (is_CLR(state)) {
           sstates.push_back(STATE_KL); 
           sstates.push_back(STATE_CLR);
        }

        return sstates;
    }

} // namespace fsm 
} // namespace

#endif // _FSM_H_