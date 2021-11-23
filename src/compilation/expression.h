#pragma once

#include <string>
#include "type.h"
#include "variable.h"

class Expression
{
public:
    Type type;
    virtual bool evaluate_boolean();
    virtual int64_t evaluate_integer();
    virtual double evaluate_number();
    virtual std::string evaluate_string();
    virtual std::string evaluate_identifier();
    bool is_numbery();
};

class BooleanExpression : public Expression
{
private:
    bool value;

public:
    BooleanExpression(bool value);
    bool evaluate_boolean();
};

class StringExpression : public Expression
{
private:
    std::string value;

public:
    StringExpression(std::string value);
    std::string evaluate_string();
};

class IntegerExpression : public Expression
{
private:
    int64_t value;

public:
    IntegerExpression(int64_t value);
    int64_t evaluate_integer();
    double evaluate_number();
};

class NumberExpression : public Expression
{
private:
    double value;

public:
    NumberExpression(double value);
    double evaluate_number();
};

class VariableExpression : public Expression
{
private:
    Variable *variable;

public:
    VariableExpression(Variable *variable);
    bool evaluate_boolean();
    int64_t evaluate_integer();
    double evaluate_number();
    std::string evaluate_string();
    std::string evaluate_identifier();
};

class PowerExpression : public Expression
{
private:
    Expression *left, *right;

public:
    PowerExpression(Expression *left, Expression *right);
    int64_t evaluate_integer();
    double evaluate_number();
};

class NegateExpression : public Expression
{
private:
    Expression *operand;

public:
    NegateExpression(Expression *operand);
    int64_t evaluate_integer();
    double evaluate_number();
};

class MultiplyExpression : public Expression
{
private:
    Expression *left, *right;

public:
    MultiplyExpression(Expression *left, Expression *right);
    int64_t evaluate_integer();
    double evaluate_number();
};

class DivideExpression : public Expression
{
private:
    Expression *left, *right;

public:
    DivideExpression(Expression *left, Expression *right);
    int64_t evaluate_integer();
    double evaluate_number();
};

class AddExpression : public Expression
{
private:
    Expression *left, *right;

public:
    AddExpression(Expression *left, Expression *right);
    int64_t evaluate_integer();
    double evaluate_number();
};

class SubtractExpression : public Expression
{
private:
    Expression *left, *right;

public:
    SubtractExpression(Expression *left, Expression *right);
    int64_t evaluate_integer();
    double evaluate_number();
};

class GreaterExpression : public Expression
{
private:
    Expression *left, *right;

public:
    GreaterExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class LessExpression : public Expression
{
private:
    Expression *left, *right;

public:
    LessExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class GreaterEqualExpression : public Expression
{
private:
    Expression *left, *right;

public:
    GreaterEqualExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class LessEqualExpression : public Expression
{
private:
    Expression *left, *right;

public:
    LessEqualExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class EqualExpression : public Expression
{
private:
    Expression *left, *right;

public:
    EqualExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class UnequalExpression : public Expression
{
private:
    Expression *left, *right;

public:
    UnequalExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class NotExpression : public Expression
{
private:
    Expression *operand;

public:
    NotExpression(Expression *operand);
    bool evaluate_boolean();
};

class AndExpression : public Expression
{
private:
    Expression *left, *right;

public:
    AndExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};

class OrExpression : public Expression
{
private:
    Expression *left, *right;

public:
    OrExpression(Expression *left, Expression *right);
    bool evaluate_boolean();
};
