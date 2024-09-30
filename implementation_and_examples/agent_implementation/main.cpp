//
// Created by hugo on 29-9-24.
//


#include <iostream>
#include <set>

int headingRounded = 31;
//int dir = 0;

// Custom comparator to order set for wall following. The set is ordered by the angle difference to the wall following direction
struct CustomComparator {
    int dir;  // dir is either 0, 1 or -1
    double headingRounded;

    CustomComparator(int dir, double headingRounded) : dir(dir), headingRounded(headingRounded) {}


    //SOMETHING GOES WRONG WITH ANGLE 122 AND HEADING 32 --> diff = 90 exactly
    //Good with heading 36 --> 86
    // Custom comparator logic
    bool operator()(const int &a, const int &b) {


        auto a_diff = a - headingRounded;
        auto b_diff = b - headingRounded;

        if(a_diff < -180) {
            a_diff += 360;
        } else if(a_diff > 179) {
            a_diff -= 360;
        }

        if(b_diff < -180) {
            b_diff += 360;
        } else if(b_diff > 179) {
            b_diff -= 360;
        }


        auto a_diff_val = a_diff;
        auto b_diff_val = b_diff;

        if (dir >= 0) {
            // Handle the first half: 90 to -180
            if (a_diff_val <= 90 && a_diff_val >= -180 && b_diff_val <= 90 && b_diff_val >= -180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Handle the second half: 180 to 91
            if (a_diff_val > 90 && a_diff_val <= 180 && b_diff_val > 90 && b_diff_val <= 180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (90 to -180) over the second half (180 to 91)
            if ((a_diff_val <= 90 && a_diff_val >= -180) && (b_diff_val > 90 && b_diff_val <= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val > 90 && a_diff_val <= 180) && (b_diff_val <= 90 && b_diff_val >= -180)) {
                return false;  // 'b' should come before 'a'
            }
        } else {
            // Handle the first half: -90 to 180
            if (a_diff_val >= -90 && a_diff_val <= 180 && b_diff_val >= -90 && b_diff_val <= 180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Handle the second half: -180 to -91
            if (a_diff_val < -90 && a_diff_val >= -180 && b_diff_val < -90 && b_diff_val >= -180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (-90 to 180) over the second half (-180 to -91)
            if ((a_diff_val >= -90 && a_diff_val <= 180) && (b_diff_val < -90 && b_diff_val >= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val < -90 && a_diff_val >= -180) && (b_diff_val >= -90 && b_diff_val <= 180)) {
                return false;  // 'b' should come before 'a'
            }
        }

        return a_diff_val > b_diff_val;  // Default to descending order if somehow unmatched
    }
};



int main() {

    std::set<int, CustomComparator> freeAngles(
            CustomComparator(0, headingRounded));

    for (int a = 0; a < 360; a++) {
        auto angle = a * 360/360 - 180;
        freeAngles.insert(angle);
    }

    for(auto angle : freeAngles) {
        auto relative = angle - headingRounded;
        if(relative < -180) {
            relative += 360;
        } else if(relative > 179) {
            relative -= 360;
        }

        std::cout << angle << " : relative : " << relative <<  std::endl;
    }
    return 0;

}
