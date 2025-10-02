#include <bits/stdc++.h>

void magic_pinning() {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    auto result = sched_setaffinity(0, sizeof(mask), &mask);
}

using namespace std;

const int BUFFER_MAX_SZ = (1 << 29);
int *bytes_buffer_shifted;
mt19937_64 rng(566);
unordered_map<int, vector<int>> orders_memory;

int bytes_buffer[2 * BUFFER_MAX_SZ]; // 4 GB...

vector<int> get_order(int spots) {
    if (orders_memory.contains(spots)) return orders_memory[spots];
    vector<int> points(spots);
    for (int i = 0; i < spots; ++i) {
        points[i] = i;
    }
    shuffle(points.begin(), points.end(), rng);
    vector<int> inv(spots);
    for (int i = 0; i < spots; ++i) {
        inv[points[i]] = i;
    }
    vector<bool> used(spots, false);
    int cur = 0;
    int fun = 0;
    for (int i = 0; i < spots; ++i) {
        used[cur] = true;
        int next = points[cur];
        if (!used[next]) {
            cur = next;
            continue;
        }
        for (; fun < spots; ++fun) {
            if (!used[fun]) {
                int s_pos = inv[fun];
                swap(points[s_pos], points[cur]);
                inv[points[cur]] = next;
                inv[points[s_pos]] = s_pos;
                break;
            }
        }
        cur = points[cur];
    }
    return orders_memory[spots] = points;
}

const int ITERS = 1 << 24;

long long perform_iterations(int stride, int spots) {
    vector<int> points = get_order(spots);
    for (int i = 0; i < spots; ++i) {
        bytes_buffer_shifted[i * stride] = points[i] * stride;
    }
    auto ptr = bytes_buffer_shifted;
    for (int j = 0; j < ITERS / 8; ++j) {
        ptr = &bytes_buffer_shifted[*ptr]; // warm up...
    }
    auto t1 = chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERS; ++i) {
        ptr = &bytes_buffer_shifted[*ptr];
    }
    auto t2 = chrono::high_resolution_clock::now();
    if (*ptr == 'z') {
        cerr << "Side effect!" << '\n';
    }
    return (chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count() * 100 + 50) / ITERS; // rounding
}

const long double coef = 1.2;

bool jump(long long old_time, long long new_time) {
    return (old_time * coef < new_time);
}

bool downward(long long old_time, long long new_time) {
    return (old_time > new_time * coef);
}

const long double wiggle_coef = 1.15;
bool are_different(int time_1, int time_2) {
    return (((long double) max(time_1, time_2)) / min(time_1, time_2)) > wiggle_coef;
}

vector<int> get_jumps_by_stride(int H, int N) {
    vector<int> jumps_prior;
    int S = 1;
    bool just_jumped = false;
    long long stored_old = 0;
    long long prev_time = perform_iterations(H, S);
    while (S < N) {
        long long cur_time = perform_iterations(H, S);
//        cerr << "H: " << H << ", S: " << S << " gets " << cur_time << '\n';
        if (just_jumped && downward(prev_time, cur_time) && !jump(stored_old, cur_time)) {
            cerr << "Got downward from " << prev_time << " to " << cur_time << ", cancelling previous jump\n";
            jumps_prior.pop_back();
        }
        just_jumped = false;
        if (jump(prev_time, cur_time)) {
            cerr << "Got Jump on " << H << ' ' << S << " from " << prev_time << " to " << cur_time << '\n';
            jumps_prior.emplace_back(S - 1);
            just_jumped = true;
            stored_old = prev_time;
        }
        S += 1;
        prev_time = cur_time;
    }
    return jumps_prior;
}

long long average_time_all_spots(int H, int L, int N) {
    int S = 1;
    long long total_time = 0;
    while (S < N) {
        long long spent = perform_iterations(H + L, S);
        total_time += spent;
        S++;
    }
    return total_time / N;
}

const int H_PW = 4;
const int H_START = 1 << H_PW;

bool is_movement(const map<long long, vector<int>> &stride_to_jump, long long cur_stride) {
    long long prev_stride = cur_stride / 2;
    if (!stride_to_jump.contains(prev_stride) && stride_to_jump.contains(cur_stride)) return true;
    if (stride_to_jump.find(prev_stride)->second.empty() && !stride_to_jump.find(cur_stride)->second.empty())
        return true;
    return stride_to_jump.find(prev_stride)->second.front() > stride_to_jump.find(cur_stride)->second.front();
}

vector<pair<int, long long>> detect_entities(const map<long long, vector<int>> &stride_to_jump) {
    auto [last_stride, last_jumps] = *stride_to_jump.rbegin();

    vector<pair<int, long long>> result;
    for (int jump_testing: last_jumps) {
        long long got_stride = -1;

        for (const auto &[stride, jumps]: ranges::reverse_view(stride_to_jump)) {
            bool found = false;
            for (int jump_new: jumps) {
                if (jump_new == jump_testing) {
                    found = true;
                    got_stride = stride;
                    break;
                }
            }

            if (!found) {
                break;
            }
        }

        if (got_stride != -1 && got_stride != last_stride) {
            result.emplace_back(jump_testing, got_stride);
        }
    }

    return result;
}

int main() {
    magic_pinning();
    int i_shift = 0;
    auto start = (long long) bytes_buffer;
    for (int i = 63; i >= 0; --i) {
        if (((1LL << i) & start) == 0 && i < 25) {
            i_shift += (1LL << i);
        }
    }
    i_shift += 1;
    bytes_buffer_shifted = (bytes_buffer + i_shift / 4);
    for (int i = 63; i >= 0; --i) {
        cerr << (((1LL << i) & ((long long) bytes_buffer_shifted)) >> i);
    }
    cerr << '\n';
    int H = H_START;
    int N = 256;
    bool started_jumping = false;
    map<long long, vector<int>> stride_to_jumps;
    while (H * N < BUFFER_MAX_SZ) {
        vector<int> jumps = get_jumps_by_stride(H, N);
        if (!jumps.empty()) {
            started_jumping = true;
        }
        stride_to_jumps[H] = jumps;
        if (started_jumping) {
            if (jumps.empty()) {
                cerr << "Stopped jumping, probably a fluke, continuing...\n";
                started_jumping = false;
            } else {
                if (!is_movement(stride_to_jumps, H)) break;
            }
        }
        H *= 2;
    }
    auto entities = detect_entities(stride_to_jumps);
    if (entities.empty()) {
        cout << "Wasn't able to find cache, too bad!\n";
        return 1;
    }
    cout << "Cache size is " << entities[0].first * entities[0].second * sizeof(int) << " bytes\n";
    cout << "Cache associativity is " << entities[0].first << '\n';

    map<int, int> trend;
    for (H = 1 << 3; H <= 1 << 10; H *= 2) {
        auto time_only_high = average_time_all_spots(H, 0LL, N);
        cerr << "Average time with only high stride " << H << ": " << time_only_high << '\n';

        int patterns[] = {0, 0}; // 'Decrease', 'Increase'
        for (int L = max(1, H >> 4); L < H; L *= 2) {
            auto time_high_low = average_time_all_spots(H, L, N);
            cerr << "Average time with low stride " << L << ": " << time_high_low << '\n';

            if (are_different(time_only_high, time_high_low)) {
                continue;
            }

            if (time_only_high > time_high_low) {
                patterns[0]++;
            } else {
                patterns[1]++;
            }
        }

        int pattern = 0;
        if (patterns[1] > patterns[0]) {
            pattern = 1;
        }
        if (patterns[0] > patterns[1]) {
            pattern = 2;
        }
        trend[H] = pattern;
    }
    cerr << "Trends: \n";
    for (const auto &e: trend) {
        cerr << "\tStride " << e.first << ": pattern " << e.second << "\n";
    }

    int line_size = -1;
    bool encountered_second = false;
    for (auto &e: trend) {
        if (e.second == 2) {
            encountered_second = true;
        }
        if (encountered_second && e.second != 2) {
            line_size = e.first / 2;
            break;
        }
    }

    if (line_size == -1) {
        cout << "Failed to estimate cache line size; too bad!\n";
        return 1;
    } else {
        cout << "Cache line size: " << line_size * sizeof(int) << " bytes.\n";
    }
    return 0;
}
