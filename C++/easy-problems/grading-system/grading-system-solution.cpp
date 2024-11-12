#include <iostream>
#include <string>

struct Student {
    std::string name;
    int score;
    char grade;
};

char calculateGrade(int score) {
    if (score >= 90)
        return 'A';
    else if (score >= 80)
        return 'B';
    else if (score >= 70)
        return 'C';
    else if (score >= 60)
        return 'D';
    else
        return 'F';
}

int main() {

    int numStudents;
    std::cout << "How many students do you have? ";
    std::cin >> numStudents;

    Student students[numStudents];

    for (int i = 0; i < numStudents; i++) {
        std::cout << "Enter name of student " << i + 1 << ": ";
        std::cin >> students[i].name;

        std::cout << "Enter score for " << students[i].name << ": ";
        std::cin >> students[i].score;

        students[i].grade = calculateGrade(students[i].score);
    }

    std::cout << "\nStudent Grades:" << std::endl;

    for (int i = 0; i < numStudents; i++) {
        std::cout << students[i].name << ": " << students[i].score << "(" << students[i].grade << ")" << std::endl;
    }

    return 0;
}