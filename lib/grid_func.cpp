#include "../Wall-E.h"

void print_grid() {
    /// Visualise the virtual grid.
    cout << "Virtual GRID state: " << endl;
    cout << "[" << endl;
    for (unsigned int y = 0; y < brain.grid.size(); y++) {
        vector<int> row;
        for (unsigned int x = 0; x < brain.grid[y].size(); x++) {
            cout << brain.grid[x][y] << ", ";
        }
        cout << endl;
    }
    cout << "]";
}

vector<int> translate_xy_to_vector(int x, int y) {
    /// Translates coordinate system coordinates to nested vector coordinates
    return {x, int(brain.grid.size()) - y - 1};
}

int get_desired_direction() {
    /// Returns desired direction based on destination and current position
    int dx = brain.destination_coordinates[0] - brain.current_coordinates[0];
    int dy = brain.destination_coordinates[1] - brain.current_coordinates[1];
    cout << "dx:" << dx << " dy:" << dy << endl;
    if (dx > dy) {
        if (dx > 0) return RIGHT;
        else if (dx < 0) return LEFT;
    } else {
        if (dy > 0) return UP;
        else if (dy < 0) return DOWN;
    }
}

int scan_surroundings() {
    /// Returns the information in the surrounding tiles
    vector<direction> dir_codes = {{RIGHT, 0}, {UP,    0},  {LEFT,  0}, {DOWN,  0}};
    vector<vector<int>> c = {{1,  0}, {0,  1}, {-1, 0}, {0, -1}};

    // Assign values to directions
    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        vector<int> vec = translate_xy_to_vector(brain.current_coordinates[0] + c[i][0],
                                                 brain.current_coordinates[1] + c[i][1]
        );

        if (vec[0] < 0 || vec[0] > brain.grid[0].size() || vec[1] < 0 || vec[1] > brain.grid.size() -1) {
            dir_codes[i].code = -1;
        } else {
            dir_codes[i].code = brain.grid[vec[0]][vec[1]];
        }
    }

    int desired_dir = get_desired_direction();
    direction tmp = dir_codes[0];
    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        cout << "dir: " << dir_codes[i].dir<< " code: " << dir_codes[i].code << " des: " << desired_dir << endl;
        if (dir_codes[i].dir == desired_dir) {
            // This is the direction you want to be going... Make sure it is not an obstacle or something
            if (dir_codes[i].code != -1 || dir_codes[i].code != 0) {
                return dir_codes[i].dir;
            }
        }
        if (dir_codes[i].code > tmp.code) {
            tmp = dir_codes[i];
        }
    }
    return tmp.dir;
}

void turn_to_destination(int direction) {
	/// Turns Wall-E on grid to next intersection.
    // 0=no change, -1=left, 1=right >1 turn
    int turn = brain.driving_direction - direction;
    if (turn == -1 or turn == 3) {
        stop_driving();
        dodge(false, -90, 0);
    } else if (turn == 1 or turn == -3) {
        stop_driving();
        dodge(false, 90, 0);
    } else if (turn == 2 or turn == -2) {
        stop_driving();
        dodge(false, 180, 0);
    }
}

void set_grid_parameters() {
	/// Sets parameters for grid.
    int x = 5, y = 5;
    cout << "Enter desired grid size as integers divided by a spece (x y): ";
    cin >> x >> y;

    for (unsigned int i = 0; i < x; i++) {
        vector<int> tmp;
        for (unsigned int j = 0; j < y; j++) {
            tmp.push_back(2);
        }
        brain.grid.push_back(tmp);
    }

    bool deciding = true;
    while (deciding) {
        cout << "Enter destination coordinates as integers divided by a space (x y): ";
        cin >> x >> y;
        if (x <= brain.grid[0].size() || x >= 0 || y <= brain.grid.size() || y >= 0) {
            deciding = false;
        }
        cout << endl;
    }
    brain.destination_coordinates = {x, y};
    brain.grid[0][translate_xy_to_vector(brain.current_coordinates[0], brain.current_coordinates[1])[1]] = 4;
    brain.grid[x][translate_xy_to_vector(0, y)[1]] = 3;

    update_virtual_grid();
}

void update_virtual_grid() {
    /// Update virtual grid based on position and driving direction
    vector<int> new_coordinates = get_new_coordinates(brain.driving_direction, brain.current_coordinates);
    cout << "new coordinates: (" << new_coordinates[0] << ", " << new_coordinates[1] << ")" << endl;
    brain.grid[translate_xy_to_vector(brain.last_coordinates[0], 0)[0]][translate_xy_to_vector(0, brain.last_coordinates[1])[1]] = 1;
    brain.current_coordinates = new_coordinates;
    if ( brain.grid[translate_xy_to_vector(brain.current_coordinates[0], 0)[0]][translate_xy_to_vector(0, brain.current_coordinates[1])[1]] == 3) {
        // Destination has been reached
        brain.found_eve = true;
    }
    brain.last_coordinates = brain.current_coordinates;
    brain.grid[translate_xy_to_vector(brain.current_coordinates[0], 0)[0]][translate_xy_to_vector(0, brain.current_coordinates[1])[1]] = 4;
}

vector<int> get_new_coordinates(int direction, vector<int> current_position) {
	/// Gets new coordinates of Wall-E in grid.
    if (direction == UP) return {current_position[0], current_position[1] + 1};
    if (direction == DOWN) return {current_position[0], current_position[1] - 1};
    if (direction == RIGHT) return {current_position[0] + 1, current_position[1]};
    if (direction == LEFT) return {current_position[0] - 1, current_position[1]};
}