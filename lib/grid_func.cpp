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

int translate_y(int y) {
    /// Translates coordinate system coordinates to nested vector coordinates.
    return brain.grid.size() - y - 1;
}

int scan_surroundings() {
    /// Returns the information in the surrounding tiles.
    vector<direction> dir_codes = {{RIGHT, 0}, {UP, 0}, {LEFT, 0},{DOWN, 0}};
    vector<vector<int>> c = {{0,  1},
                             {0,  -1},
                             {-1, 0},
                             {-1, 0}};

    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        int x = brain.current_coordinates[0] + c[i][0];
        int y = brain.current_coordinates[1] + translate_y(c[i][1]);

        if (x < 0 || x > brain.grid[0].size() || y < 0 || y > brain.grid.size()) {
            dir_codes[i].code = -1;
        } else  {
            dir_codes[i].code = brain.grid[x][y];
        }
    }

    direction tmp;
    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        tmp = dir_codes[0];
        if (dir_codes[i].code > tmp.code) {
            if (dir_codes[i].code != 4) {
                tmp = dir_codes[i];
            }
        }
        if (tmp.code == 3) {
            cout << "E.V.E. found!" << endl;
            dodge(0, 180, 0);
            brain.exit = true;
        }
    }

    print_grid();

    return tmp.dir;
}

void turn_to_destination(int direction) {
	/// Turns Wall-E on grid to next intersection.
    // 0=no change, -1=left, 1=right >1 turn
    int turn = brain.driving_direction - direction;
    if (turn == -1 or turn == 3) {
        dodge(0, 90, 0);
    } else if (turn == 1 or turn == -3) {
        dodge(0, -90, 0);
    } else if (turn == 2 or turn == -2) {
        dodge(0, 180, 0);
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
    brain.grid[0][translate_y(0)] = 4;
    brain.grid[x][translate_y(y)] = 3;
}

void update_virtual_grid() {
    /// Update virtual grid based on position and driving direction
    // Set current position value to 1 for explored.
    int cx = brain.current_coordinates[0];
    int cy = brain.current_coordinates[1];
    int vx = cx;
    int vy = translate_y(cy);
    brain.grid[vx][vy] = 1;

    // Set new position for Wall-E based on brain.driving direction
    brain.last_coordinates = brain.current_coordinates;
    brain.current_coordinates = get_new_coordinates(brain.driving_direction, brain.current_coordinates);
    brain.grid[brain.current_coordinates[0]][translate_y(brain.current_coordinates[1])] = 4;
}

vector<int> get_new_coordinates(int direction, vector<int> current_position) {
	/// Gets new coordinates of Wall-E in grid.
    if (direction == UP) return {current_position[0], current_position[1] + 1};
    if (direction == DOWN) return {current_position[0], current_position[1] - 1};
    if (direction == RIGHT) return {current_position[0] + 1, current_position[1]};
    if (direction == LEFT) return {current_position[0] - 1, current_position[1]};
}