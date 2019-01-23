#ifndef _FSM_H_
#define _FSM_H_

#include <vector>
#include <string>

using namespace std;

namespace fsm {

    string STATE_KL = "KL"; // keep lane
    string STATE_CLL = "CLL"; // change lane left
    string STATE_CLR = "CLR"; // change lane right

    vector<string> successor_states(const string& state) {
        vector<string> sstates;

        if (state.compare(STATE_KL) == 0) {
            sstates.push_back(STATE_KL);
            sstates.push_back(STATE_CLL);
            sstates.push_back(STATE_CLR);
        }
        else if (state.compare(STATE_CLL)) {
           sstates.push_back(STATE_KL); 
        }
        else if (state.compare(STATE_CLR)) {
           sstates.push_back(STATE_KL); 
        }

        return sstates;
    }

} // namespace fsm 

#endif // _FSM_H_