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

    for (; ; ) { // populate initialization, condition, and increment sections
        // Implement loop body to populate students with name, score, and grade
    }

    std::cout << "\nStudent Grades:" << std::endl;

    for (; ; ) { // populate initialization, condition, and increment sections
        // Implement loop body to output contents 
    }

    return 0;
}
