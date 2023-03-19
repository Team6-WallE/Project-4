from rest_framework import serializers
from .models import QuestionTable, AnswerTable


class QuestionSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = QuestionTable
        fields = ('question_id', 'questions', 'url')

class AnswerSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = AnswerTable
        fields = ('question_id', 'answers', 'url')