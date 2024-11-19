#include <iostream>
#include <string>
#include <vector>

struct Student {
    std::string name;
    int score;
    char grade;

    Student(std::string name, int score, char grade) : name(name), score(score), grade(grade) {}
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

    std::vector<Student> students;

    for (int i = 0; i < numStudents; i++) {
        std::string name;
        int score;
        char grade;

        std::cout << "Enter name of student " << i + 1 << ": ";
        std::cin >> name;

        std::cout << "Enter score for " << name << ": ";
        std::cin >> score;

        grade = calculateGrade(score);

        students.push_back(Student(name, score, grade));
    }

    std::cout << "\nStudent Grades:" << std::endl;

    for (int i = 0; i < numStudents; i++) {
        std::cout << students[i].name << ": " << students[i].score << "(" << students[i].grade << ")" << std::endl;
    }

    return 0;
}
